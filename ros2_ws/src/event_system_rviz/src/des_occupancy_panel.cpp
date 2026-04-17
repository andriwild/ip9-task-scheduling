#include <event_system_rviz/des_occupancy_panel.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace des_occupancy_panel {

DesOccupancyPanel::DesOccupancyPanel(QWidget* parent) : rviz_common::Panel(parent) {
    auto mainLayout = new QVBoxLayout(this);

    m_axisX = new QtCharts::QValueAxis();
    m_axisX->setTitleText("Hour");
    m_axisX->setRange(0, 24);
    m_axisX->setTickCount(25);
    m_axisX->setLabelFormat("%.0f");
    m_axisX->setGridLineVisible(true);
    m_axisX->setMinorTickCount(0);

    m_axisY = new QtCharts::QCategoryAxis();
    m_axisY->setTitleText("Location");
    m_axisY->setRange(0, 1);
    m_axisY->setStartValue(0);
    m_axisY->append("OUTDOOR", 0);
    m_roomIndexMap["OUTDOOR"] = 0;

    m_axisY->setLabelsPosition(QtCharts::QCategoryAxis::AxisLabelsPositionOnValue);

    m_chart = new QtCharts::QChart();
    m_chart->addAxis(m_axisX, Qt::AlignBottom);
    m_chart->addAxis(m_axisY, Qt::AlignLeft);
    m_chart->setTitle("Person Occupancy");
    m_chart->legend()->setAlignment(Qt::AlignTop);
    m_chart->setMargins(QMargins(100, 5, 5, 5));

    m_chartView = new InteractiveChartView(m_chart);
    m_chartView->setRenderHint(QPainter::Antialiasing);
    m_chartView->setRubberBand(QtCharts::QChartView::HorizontalRubberBand);
    m_chartView->setMinimumHeight(350);

    mainLayout->addWidget(m_chartView);
}

void DesOccupancyPanel::onInitialize() {
    auto node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
    m_node = node_abstraction->get_raw_node();

    auto qos = rclcpp::QoS(rclcpp::KeepAll()).reliable().transient_local();

    m_subEvent = m_node->create_subscription<event_system_msgs::msg::TimelineEvent>(
        "/timeline/event", qos,
        [this](const event_system_msgs::msg::TimelineEvent::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() { this->onTimelineEvent(msg); });
        });

    m_subReset = m_node->create_subscription<event_system_msgs::msg::TimelineReset>(
        "/timeline/reset", qos,
        [this](const event_system_msgs::msg::TimelineReset::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() { this->onReset(msg); });
        });
}

void DesOccupancyPanel::onTimelineEvent(const event_system_msgs::msg::TimelineEvent::SharedPtr msg) {
    const uint8_t type = msg->type;

    if (type == event_system_msgs::msg::TimelineEvent::SIMULATION_START) {
        m_simStart = msg->time;
        updateXAxis();
        return;
    }
    if (type == event_system_msgs::msg::TimelineEvent::SIMULATION_END) {
        m_simEnd = msg->time;
        updateXAxis();
        return;
    }

    // Robot: START_DRIVE extends horizontal line to departure time,
    //        STOP_DRIVE draws diagonal to arrival (location + time)
    if (type == event_system_msgs::msg::TimelineEvent::START_DRIVE) {
        addRobotDeparture(msg->time);
        return;
    }
    if (type == event_system_msgs::msg::TimelineEvent::STOP_DRIVE) {
        QString label = QString::fromStdString(msg->label);
        static const QString prefix = "Arrived: ";
        if (!label.startsWith(prefix)) return;
        QString room = label.mid(prefix.length());
        addRobotArrival(room, msg->time);
        return;
    }

    if (type == event_system_msgs::msg::TimelineEvent::PERSON_ACCOMPANY_DEPARTURE) {
        auto [person, room] = parseLabel(QString::fromStdString(msg->label));
        if (person.isEmpty()) return;
        addPersonDeparture(person, room, msg->time, QString::fromStdString(msg->color));
        return;
    }

    if (type == event_system_msgs::msg::TimelineEvent::PERSON_ACCOMPANY_ARRIVED) {
        auto [person, room] = parseLabel(QString::fromStdString(msg->label));
        if (person.isEmpty()) return;
        addPersonArrival(person, room, msg->time, QString::fromStdString(msg->color));
        return;
    }

    const bool isPersonEvent =
        type == event_system_msgs::msg::TimelineEvent::PERSON_TRANSITION ||
        type == event_system_msgs::msg::TimelineEvent::PERSON_ARRIVED ||
        type == event_system_msgs::msg::TimelineEvent::PERSON_DEPARTURE;

    if (!isPersonEvent) return;

    auto [person, room] = parseLabel(QString::fromStdString(msg->label));
    if (person.isEmpty()) return;

    addPoint(person, room, msg->time, QString::fromStdString(msg->color));
}

void DesOccupancyPanel::onReset(const event_system_msgs::msg::TimelineReset::SharedPtr /*msg*/) {
    for (auto* s : m_seriesMap.values()) {
        m_chart->removeSeries(s);
        delete s;
    }
    m_seriesMap.clear();
    m_lastY.clear();
    m_roomIndexMap.clear();

    // Reset Y axis
    for (const auto& label : m_axisY->categoriesLabels()) {
        m_axisY->remove(label);
    }
    m_axisY->setRange(0, 1);
    m_axisY->setStartValue(0);
    m_axisY->append("OUTDOOR", 0);
    m_roomIndexMap["OUTDOOR"] = 0;
    m_nextRoomIndex = 1;

    m_simStart = 0;
    m_simEnd = 86400;
    m_axisX->setRange(0, 24);
}

std::pair<QString, QString> DesOccupancyPanel::parseLabel(const QString& label) const {
    // Labels: "{name} moved to {room}", "{name} arrived to {room}", "{name} leaved to {room}"
    static const QStringList patterns = {" moved to ", " arrived to ", " leaved to "};
    for (const auto& pat : patterns) {
        int idx = label.indexOf(pat);
        if (idx >= 0) {
            QString person = label.left(idx);
            QString room = label.mid(idx + pat.length());
            return {person, room};
        }
    }
    return {{}, {}};
}

int DesOccupancyPanel::roomIndex(const QString& room) {
    if (m_roomIndexMap.contains(room)) {
        return m_roomIndexMap[room];
    }

    int idx = m_nextRoomIndex++;
    m_roomIndexMap[room] = idx;
    m_axisY->append(room, idx);
    m_axisY->setRange(0, m_nextRoomIndex);
    return idx;
}

QtCharts::QLineSeries* DesOccupancyPanel::getOrCreateSeries(const QString& person, const QString& color) {
    if (m_seriesMap.contains(person)) {
        return m_seriesMap[person];
    }

    auto* series = new QtCharts::QLineSeries();
    series->setName(person);

    QPen pen{QColor{color}};
    pen.setWidth(2);
    series->setPen(pen);

    m_chart->addSeries(series);
    series->attachAxis(m_axisX);
    series->attachAxis(m_axisY);

    connectSeriesSignals(series);
    connectLegendMarkers();

    m_seriesMap[person] = series;
    return series;
}

void DesOccupancyPanel::connectSeriesSignals(QtCharts::QLineSeries* series) {
    connect(series, &QtCharts::QLineSeries::hovered, this,
        [this](const QPointF& point, bool state) {
            if (!state) {
                QToolTip::hideText();
                return;
            }
            // Resolve room name from Y index
            int yIdx = qRound(point.y());
            QString room;
            for (auto it = m_roomIndexMap.constBegin(); it != m_roomIndexMap.constEnd(); ++it) {
                if (it.value() == yIdx) { room = it.key(); break; }
            }
            // Format time as HH:MM
            int totalSeconds = static_cast<int>(point.x() * 3600);
            int h = totalSeconds / 3600;
            int m = (totalSeconds % 3600) / 60;
            QString tip = QString("%1:%2 — %3").arg(h, 2, 10, QChar('0')).arg(m, 2, 10, QChar('0')).arg(room);
            QToolTip::showText(QCursor::pos(), tip, m_chartView);
        });
}

void DesOccupancyPanel::connectLegendMarkers() {
    for (auto* marker : m_chart->legend()->markers()) {
        // Disconnect first to avoid duplicate connections
        disconnect(marker, &QtCharts::QLegendMarker::clicked, nullptr, nullptr);
        connect(marker, &QtCharts::QLegendMarker::clicked, this, [marker]() {
            auto* series = marker->series();
            bool visible = !series->isVisible();
            series->setVisible(visible);
            marker->setVisible(true);  // Keep marker always visible
            // Dim the marker label/icon when series is hidden
            auto brush = marker->labelBrush();
            auto color = brush.color();
            color.setAlphaF(visible ? 1.0 : 0.4);
            brush.setColor(color);
            marker->setLabelBrush(brush);
        });
    }
}

void DesOccupancyPanel::addRobotDeparture(int time) {
    if (!m_lastY.contains("Robot")) return;
    const double hours = time / 3600.0;
    auto* series = getOrCreateSeries("Robot", "#E00000");
    // Extend horizontal line to the departure time at current location
    series->append(hours, m_lastY["Robot"]);
}

void DesOccupancyPanel::addRobotArrival(const QString& room, int time) {
    // Diagonal line: previous point (departure) connects directly to arrival
    const double hours = time / 3600.0;
    const int yIdx = roomIndex(room);
    auto* series = getOrCreateSeries("Robot", "#E00000");
    series->append(hours, yIdx);
    m_lastY["Robot"] = yIdx;
}

void DesOccupancyPanel::addPersonDeparture(const QString& name, const QString& room, int time, const QString& color) {
    const int yIdx = roomIndex(room);
    const double hours = time / 3600.0;
    auto* series = getOrCreateSeries(name, color);
    // Step to current room first (if the last known Y differs), then anchor at departure time.
    if (m_lastY.contains(name) && m_lastY[name] != yIdx) {
        series->append(hours, m_lastY[name]);
    }
    series->append(hours, yIdx);
    m_lastY[name] = yIdx;
}

void DesOccupancyPanel::addPersonArrival(const QString& name, const QString& room, int time, const QString& color) {
    const double hours = time / 3600.0;
    const int yIdx = roomIndex(room);
    auto* series = getOrCreateSeries(name, color);
    series->append(hours, yIdx);
    m_lastY[name] = yIdx;
}

void DesOccupancyPanel::addPoint(const QString& name, const QString& room, int time, const QString& color) {
    const double hours = time / 3600.0;
    const int yIdx = roomIndex(room);

    auto* series = getOrCreateSeries(name, color);

    // Step-line: extend horizontal at previous Y, then jump vertical
    if (m_lastY.contains(name)) {
        series->append(hours, m_lastY[name]);
    }
    series->append(hours, yIdx);
    m_lastY[name] = yIdx;
}

void DesOccupancyPanel::updateXAxis() {
    int startHour = static_cast<int>(m_simStart / 3600.0);
    int endHour = static_cast<int>(std::ceil(m_simEnd / 3600.0));
    int span = endHour - startHour;
    if (span < 1) span = 1;
    m_axisX->setRange(startHour, endHour);
    m_axisX->setTickCount(span + 1);
}

}  // namespace des_occupancy_panel

PLUGINLIB_EXPORT_CLASS(des_occupancy_panel::DesOccupancyPanel, rviz_common::Panel)
