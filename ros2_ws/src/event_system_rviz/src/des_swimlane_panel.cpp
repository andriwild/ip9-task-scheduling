#include <event_system_rviz/des_swimlane_panel.hpp>

#include <QBrush>
#include <QGraphicsLineItem>
#include <QGraphicsPathItem>
#include <QGraphicsRectItem>
#include <QGraphicsSimpleTextItem>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidgetItem>
#include <QMouseEvent>
#include <QPainterPath>
#include <QPen>
#include <QKeyEvent>
#include <QResizeEvent>
#include <QScrollBar>
#include <QWheelEvent>
#include <QSplitter>
#include <QTimer>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <limits>
#include <rviz_common/display_context.hpp>
#include <unordered_set>

namespace des_swimlane_panel {

namespace {

// Layout constants -----------------------------------------------------------
constexpr int kPxPerSecondInit  = 1;    // scene units == seconds; view scales
constexpr int kHeaderHeight     = 24;   // axis row at top
constexpr int kLaneHeightSystem  = 32;
constexpr int kLaneHeightMission = 54;
constexpr int kLaneHeightPerson  = 30;
constexpr int kLaneHeightState   = 32;
constexpr int kBatteryLaneHeight = 70;
constexpr int kLabelColumnWidth = 150;  // sticky labels widget (not in scene)
constexpr int kSpanHeight       = 22;
constexpr int kPadTime          = 600;  // 10 min padding around data

// Lane IDs (logical) ---------------------------------------------------------
const QString kLaneSystem        = "system";
const QString kLaneMissionBg     = "mission:background";
const QString kLaneMissionSched  = "mission:scheduled";
const QString kLaneMissionInt    = "mission:interrupt";
const QString kLaneState         = "state";
const QString kLaneBattery       = "battery";
const QString kLanePersonPrefix  = "person:";

// Match des::ExecutionMode ordering and TimelineMeeting.msg constants.
constexpr quint8 kExecScheduled  = 0;
constexpr quint8 kExecBackground = 1;
constexpr quint8 kExecInterrupt  = 2;

QString missionLaneIdFor(quint8 mode) {
    switch (mode) {
        case kExecScheduled: return kLaneMissionSched;
        case kExecInterrupt: return kLaneMissionInt;
        case kExecBackground:
        default:             return kLaneMissionBg;
    }
}

QString missionLaneLabelFor(quint8 mode) {
    switch (mode) {
        case kExecScheduled: return "Missions (Scheduled)";
        case kExecInterrupt: return "Missions (Interrupt)";
        case kExecBackground:
        default:             return "Missions (Background)";
    }
}

// Color for lane backgrounds (alternating) ----------------------------------
QColor laneBg(int row) {
    return (row % 2 == 0) ? QColor(245, 245, 248) : QColor(232, 234, 238);
}

// Colors aligned with event_system_rviz/timeline/timeline_types.hpp (DesTimelinePanel)
QColor stateColor(quint8 s) {
    using SC = event_system_msgs::msg::TimelineStateChange;
    switch (s) {
        case SC::IDLE:       return QColor(200, 200, 200);
        case SC::SEARCHING:  return QColor(255, 100, 100);
        case SC::CONVERSATE: return QColor(180, 130, 220);
        case SC::ACCOMPANY:  return QColor(100, 180, 255);
        case SC::CHARGING:   return QColor(255, 210, 50);
        case SC::RETURNING:  return QColor(120, 200, 160);
        default:             return Qt::gray;
    }
}

QString humanTime(int sec) {
    int h = sec / 3600;
    int m = (sec % 3600) / 60;
    int s = sec % 60;
    return QString::asprintf("%02d:%02d:%02d", h, m, s);
}

bool isMissionEvent(quint8 t) {
    using TE = event_system_msgs::msg::TimelineEvent;
    return t == TE::MISSION_DISPATCH
        || t == TE::MISSION_START
        || t == TE::MISSION_COMPLETE
        || t == TE::START_DRIVE
        || t == TE::STOP_DRIVE
        || t == TE::START_ACCOMPANY
        || t == TE::ARRIVED_ACCOMPANY
        || t == TE::START_DROP_OFF_CONV
        || t == TE::START_FOUND_PERSON_CONV
        || t == TE::DROP_OFF_CONV_COMPLETE
        || t == TE::FOUND_PERSON_CONV_COMPLETE
        || t == TE::ABORT_SEARCH;
}

bool isPersonEvent(quint8 t) {
    using TE = event_system_msgs::msg::TimelineEvent;
    return t == TE::PERSON_TRANSITION
        || t == TE::PERSON_ARRIVED
        || t == TE::PERSON_DEPARTURE
        || t == TE::PERSON_ACCOMPANY_ARRIVED
        || t == TE::PERSON_ACCOMPANY_DEPARTURE;
}

bool isSystemEvent(quint8 t) {
    using TE = event_system_msgs::msg::TimelineEvent;
    return t == TE::SIMULATION_START
        || t == TE::SIMULATION_END
        || t == TE::BATTERY_FULL
        || t == TE::RESET;
}

// Pull "Name " from labels like "Nina moved to Kitchen" --------------------
QString extractPersonName(const QString& label) {
    int sp = label.indexOf(' ');
    if (sp <= 0) return {};
    return label.left(sp);
}

// ---------------- Event Glyphs ----------------------------------------------

enum class Glyph { Bar, Square, Circle, Diamond, TriangleRight, TriangleLeft, Cross };

struct GlyphSpec {
    Glyph glyph;
    int sizePx;
};

GlyphSpec glyphFor(quint8 type) {
    using TE = event_system_msgs::msg::TimelineEvent;
    switch (type) {
        // "Start" events → right triangle
        case TE::START_DRIVE:
        case TE::START_ACCOMPANY:
        case TE::START_DROP_OFF_CONV:
        case TE::START_FOUND_PERSON_CONV:
            return {Glyph::TriangleRight, 12};

        // "End / arrival" events → left triangle
        case TE::STOP_DRIVE:
        case TE::ARRIVED_ACCOMPANY:
        case TE::DROP_OFF_CONV_COMPLETE:
        case TE::FOUND_PERSON_CONV_COMPLETE:
            return {Glyph::TriangleLeft, 12};

        // Mission lifecycle → diamonds
        case TE::MISSION_DISPATCH: return {Glyph::Diamond, 10};
        case TE::MISSION_START:    return {Glyph::Diamond, 14};
        case TE::MISSION_COMPLETE: return {Glyph::Diamond, 14};

        // System-wide markers → tall bars / squares
        case TE::SIMULATION_START:
        case TE::SIMULATION_END:
        case TE::RESET:
            return {Glyph::Bar, 22};
        case TE::BATTERY_FULL: return {Glyph::Square, 12};

        // Faulty / abort → cross
        case TE::ABORT_SEARCH: return {Glyph::Cross, 14};

        // Person events — Arrival = ◀ (incoming, like STOP_DRIVE),
        // Departure = ▶ (outgoing, like START_DRIVE).
        case TE::PERSON_ARRIVED:
        case TE::PERSON_ACCOMPANY_ARRIVED:
            return {Glyph::TriangleLeft, 10};
        case TE::PERSON_DEPARTURE:
        case TE::PERSON_ACCOMPANY_DEPARTURE:
            return {Glyph::TriangleRight, 10};
        case TE::PERSON_TRANSITION: return {Glyph::Circle, 8};
        default:                    return {Glyph::Bar, 14};
    }
}

QGraphicsItem* makeGlyphItem(QGraphicsScene* scene, const GlyphSpec& g,
                             const QColor& fill, const QColor& edge) {
    const qreal h = g.sizePx;
    const qreal hh = h / 2.0;
    QPen pen(edge);
    pen.setCosmetic(true);
    QBrush br(fill);
    QGraphicsItem* item = nullptr;
    switch (g.glyph) {
        case Glyph::Bar:
            item = scene->addRect(-1.0, -hh, 2.0, h, pen, br);
            break;
        case Glyph::Square:
            item = scene->addRect(-hh, -hh, h, h, pen, br);
            break;
        case Glyph::Circle:
            item = scene->addEllipse(-hh, -hh, h, h, pen, br);
            break;
        case Glyph::Diamond: {
            QPolygonF poly;
            poly << QPointF(0, -hh) << QPointF(hh, 0)
                 << QPointF(0,  hh) << QPointF(-hh, 0);
            item = scene->addPolygon(poly, pen, br);
            break;
        }
        case Glyph::TriangleRight: {
            QPolygonF poly;
            poly << QPointF(-hh, -hh) << QPointF(hh, 0) << QPointF(-hh, hh);
            item = scene->addPolygon(poly, pen, br);
            break;
        }
        case Glyph::TriangleLeft: {
            QPolygonF poly;
            poly << QPointF(hh, -hh) << QPointF(-hh, 0) << QPointF(hh, hh);
            item = scene->addPolygon(poly, pen, br);
            break;
        }
        case Glyph::Cross: {
            QPainterPath p;
            p.moveTo(-hh, -hh); p.lineTo(hh, hh);
            p.moveTo(hh, -hh);  p.lineTo(-hh, hh);
            QPen cp(edge);
            cp.setWidth(2);
            cp.setCosmetic(true);
            item = scene->addPath(p, cp);
            break;
        }
    }
    if (item) item->setFlag(QGraphicsItem::ItemIgnoresTransformations, true);
    return item;
}

// Sub-band within a lane (0..1 = vertical fraction within lane height)
double subBandFor(quint8 type) {
    using TE = event_system_msgs::msg::TimelineEvent;
    switch (type) {
        // Start / departure events → upper third
        case TE::START_DRIVE:
        case TE::START_ACCOMPANY:
        case TE::START_DROP_OFF_CONV:
        case TE::START_FOUND_PERSON_CONV:
        case TE::MISSION_DISPATCH:
        case TE::MISSION_START:
        case TE::PERSON_DEPARTURE:
        case TE::PERSON_ACCOMPANY_DEPARTURE:
            return 0.28;
        // End / arrival → lower third
        case TE::STOP_DRIVE:
        case TE::ARRIVED_ACCOMPANY:
        case TE::DROP_OFF_CONV_COMPLETE:
        case TE::FOUND_PERSON_CONV_COMPLETE:
        case TE::MISSION_COMPLETE:
        case TE::PERSON_ARRIVED:
        case TE::PERSON_ACCOMPANY_ARRIVED:
            return 0.72;
        default:
            return 0.50;
    }
}

}  // namespace

// ====================== SwimlaneView ========================================

SwimlaneView::SwimlaneView(QWidget* parent)
    : QGraphicsView(parent)
{
    setRenderHint(QPainter::Antialiasing, true);
    setDragMode(QGraphicsView::ScrollHandDrag);
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    setResizeAnchor(QGraphicsView::AnchorViewCenter);
    setAlignment(Qt::AlignLeft | Qt::AlignTop);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    setBackgroundBrush(QColor(250, 250, 252));
    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);
}

qreal SwimlaneView::xScale() const {
    return transform().m11();
}

void SwimlaneView::resetZoom() {
    QTransform t;
    setTransform(t);
    if (m_onViewportChanged) m_onViewportChanged();
}

void SwimlaneView::zoomXAt(qreal factor, QPoint viewportAnchor) {
    // Clamp absolute zoom: from 0.01x (very wide) to 200x (very tight)
    qreal current = xScale();
    qreal next = current * factor;
    if (next < 0.01) factor = 0.01 / current;
    if (next > 200.0) factor = 200.0 / current;
    if (qFuzzyCompare(factor, 1.0)) return;

    // Save scene point under the cursor
    QPointF anchorScene = mapToScene(viewportAnchor);
    // Scale only X
    QTransform t = transform();
    t.scale(factor, 1.0);
    setTransform(t);
    // Restore: scroll so anchorScene maps back to viewportAnchor.x
    QPointF newViewportPos = mapFromScene(anchorScene);
    int dx = int(newViewportPos.x()) - viewportAnchor.x();
    horizontalScrollBar()->setValue(horizontalScrollBar()->value() + dx);
}

void SwimlaneView::wheelEvent(QWheelEvent* e) {
    const Qt::KeyboardModifiers mods = e->modifiers();
    const int dy = e->angleDelta().y();
    const int dx = e->angleDelta().x();
    if (dy == 0 && dx == 0) { e->ignore(); return; }

    if (mods & Qt::ControlModifier) {
        // Zoom horizontal with anchor under cursor
        const qreal step = 1.15;
        qreal factor = (dy > 0) ? step : (1.0 / step);
        zoomXAt(factor, e->position().toPoint());
        e->accept();
    } else if (mods & Qt::ShiftModifier) {
        // Vertical scroll (default Qt behavior — let parent handle it)
        QGraphicsView::wheelEvent(e);
        return;
    } else {
        // Horizontal scroll (wheel Y maps to horizontal scrollbar)
        QScrollBar* h = horizontalScrollBar();
        // Use pixelDelta if available (precise touchpads), else angleDelta
        int amount = e->pixelDelta().x() != 0 ? e->pixelDelta().x()
                    : (e->pixelDelta().y() != 0 ? e->pixelDelta().y() : dy);
        h->setValue(h->value() - amount);
        e->accept();
    }
}

void SwimlaneView::scrollContentsBy(int dx, int dy) {
    QGraphicsView::scrollContentsBy(dx, dy);
    if (m_onViewportChanged) m_onViewportChanged();
}

void SwimlaneView::keyPressEvent(QKeyEvent* e) {
    switch (e->key()) {
        case Qt::Key_Plus:
        case Qt::Key_Equal:
            zoomXAt(1.25, QPoint(viewport()->width() / 2, viewport()->height() / 2));
            e->accept();
            return;
        case Qt::Key_Minus:
            zoomXAt(1.0 / 1.25, QPoint(viewport()->width() / 2, viewport()->height() / 2));
            e->accept();
            return;
        case Qt::Key_0:
            if (e->modifiers() & Qt::ControlModifier) {
                resetZoom();
                e->accept();
                return;
            }
            break;
        case Qt::Key_Home:
            horizontalScrollBar()->setValue(horizontalScrollBar()->minimum());
            e->accept();
            return;
        case Qt::Key_End:
            horizontalScrollBar()->setValue(horizontalScrollBar()->maximum());
            e->accept();
            return;
        default:
            break;
    }
    QGraphicsView::keyPressEvent(e);
}

// ====================== MinimapView =========================================

MinimapView::MinimapView(QWidget* parent)
    : QGraphicsView(parent)
{
    setScene(&m_scene);
    setRenderHint(QPainter::Antialiasing, false);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setFixedHeight(60);
    setFrameShape(QFrame::StyledPanel);
    setMouseTracking(true);
    setBackgroundBrush(QColor(250, 250, 252));
}

void MinimapView::setSimRange(int tMin, int tMax) {
    m_tMin = tMin;
    m_tMax = std::max(tMax, tMin + 1);
    rebuildHistogram();
}

void MinimapView::setEventTimes(const std::vector<int>& times) {
    m_times = times;
    rebuildHistogram();
}

void MinimapView::setBrush(int tBrushStart, int tBrushEnd) {
    m_brushStart = std::clamp(tBrushStart, m_tMin, m_tMax);
    m_brushEnd   = std::clamp(tBrushEnd,   m_tMin, m_tMax);
    if (m_brushEnd < m_brushStart + 60) m_brushEnd = m_brushStart + 60;
    rebuildHistogram();
}

void MinimapView::setOnBrushChanged(std::function<void(int, int)> cb) {
    m_onBrushChanged = std::move(cb);
}

void MinimapView::resizeEvent(QResizeEvent* e) {
    QGraphicsView::resizeEvent(e);
    rebuildHistogram();
}

int MinimapView::xToTime(int x) const {
    const int w = std::max(1, viewport()->width());
    const double frac = std::clamp(double(x) / w, 0.0, 1.0);
    return m_tMin + int(frac * (m_tMax - m_tMin));
}

void MinimapView::rebuildHistogram() {
    m_scene.clear();
    m_brushItem = nullptr;

    const int w = std::max(50, viewport()->width());
    const int h = viewport()->height();
    m_scene.setSceneRect(0, 0, w, h);

    if (m_tMax <= m_tMin) return;

    const int bucketCount = std::min(w, 240);
    std::vector<int> buckets(bucketCount, 0);
    for (int t : m_times) {
        if (t < m_tMin || t > m_tMax) continue;
        int idx = int(double(t - m_tMin) / (m_tMax - m_tMin) * (bucketCount - 1));
        idx = std::clamp(idx, 0, bucketCount - 1);
        buckets[idx] += 1;
    }
    int peak = 1;
    for (int v : buckets) peak = std::max(peak, v);
    const double bw = double(w) / bucketCount;
    QPen p(QColor("#5a7"));
    p.setCosmetic(true);
    QBrush br(QColor("#94c7a8"));
    for (int i = 0; i < bucketCount; ++i) {
        if (buckets[i] == 0) continue;
        double bh = double(buckets[i]) / peak * (h - 4);
        m_scene.addRect(i * bw, h - bh - 2, bw, bh, p, br);
    }

    // Brush rect
    auto timeToX = [&](int t) {
        return double(t - m_tMin) / (m_tMax - m_tMin) * w;
    };
    double bx1 = timeToX(m_brushStart);
    double bx2 = timeToX(m_brushEnd);
    QPen brushPen(QColor(60, 90, 200));
    brushPen.setWidth(2);
    brushPen.setCosmetic(true);
    QBrush brushFill(QColor(60, 90, 200, 60));
    m_brushItem = m_scene.addRect(bx1, 0, bx2 - bx1, h, brushPen, brushFill);
}

void MinimapView::mousePressEvent(QMouseEvent* e) {
    if (e->button() != Qt::LeftButton || m_tMax <= m_tMin) return;
    const int x = e->pos().x();
    const int t = xToTime(x);

    auto timeToX = [&](int tv) {
        const int w = std::max(1, viewport()->width());
        return double(tv - m_tMin) / (m_tMax - m_tMin) * w;
    };
    const double bx1 = timeToX(m_brushStart);
    const double bx2 = timeToX(m_brushEnd);
    constexpr int edgeTol = 6;
    if (std::abs(x - bx1) <= edgeTol) {
        m_drag = Drag::LeftEdge;
    } else if (std::abs(x - bx2) <= edgeTol) {
        m_drag = Drag::RightEdge;
    } else if (x > bx1 && x < bx2) {
        m_drag = Drag::Brush;
        m_dragOffset = t - m_brushStart;
    } else {
        // Click outside: recenter brush on click
        const int width = m_brushEnd - m_brushStart;
        m_brushStart = std::clamp(t - width / 2, m_tMin, m_tMax - width);
        m_brushEnd = m_brushStart + width;
        m_drag = Drag::Brush;
        m_dragOffset = t - m_brushStart;
        rebuildHistogram();
        if (m_onBrushChanged) m_onBrushChanged(m_brushStart, m_brushEnd);
    }
}

void MinimapView::mouseMoveEvent(QMouseEvent* e) {
    if (m_drag == Drag::None) return;
    const int t = xToTime(e->pos().x());
    if (m_drag == Drag::LeftEdge) {
        m_brushStart = std::clamp(t, m_tMin, m_brushEnd - 60);
    } else if (m_drag == Drag::RightEdge) {
        m_brushEnd = std::clamp(t, m_brushStart + 60, m_tMax);
    } else if (m_drag == Drag::Brush) {
        const int width = m_brushEnd - m_brushStart;
        m_brushStart = std::clamp(t - m_dragOffset, m_tMin, m_tMax - width);
        m_brushEnd = m_brushStart + width;
    }
    rebuildHistogram();
    if (m_onBrushChanged) m_onBrushChanged(m_brushStart, m_brushEnd);
}

void MinimapView::mouseReleaseEvent(QMouseEvent*) {
    m_drag = Drag::None;
}

// ====================== DesSwimlanePanel ====================================

DesSwimlanePanel::DesSwimlanePanel(QWidget* parent) : Panel(parent) {
    auto* root = new QHBoxLayout(this);
    root->setContentsMargins(4, 4, 4, 4);

    // Left side: (sticky label column + view) above, minimap below
    auto* leftLayout = new QVBoxLayout;
    leftLayout->setContentsMargins(0, 0, 0, 0);
    leftLayout->setSpacing(0);

    auto* mainRow = new QHBoxLayout;
    mainRow->setContentsMargins(0, 0, 0, 0);
    mainRow->setSpacing(0);

    // Sticky lane-label column (does NOT scroll horizontally with the view)
    m_labelColumn = new QWidget(this);
    m_labelColumn->setFixedWidth(kLabelColumnWidth);
    m_labelLayout = new QVBoxLayout(m_labelColumn);
    m_labelLayout->setContentsMargins(0, 0, 0, 0);
    m_labelLayout->setSpacing(0);
    m_labelColumn->setStyleSheet("background: #e8eaee;");

    m_scene = new QGraphicsScene(this);
    m_view = new SwimlaneView(this);
    m_view->setScene(m_scene);

    mainRow->addWidget(m_labelColumn);
    mainRow->addWidget(m_view, 1);
    leftLayout->addLayout(mainRow, 1);

    m_minimap = new MinimapView(this);
    m_minimap->setOnBrushChanged([this](int t0, int t1) {
        if (m_syncing) return;
        m_brushStart = t0;
        m_brushEnd   = t1;
        syncViewFromBrush();
    });
    m_view->setOnViewportChanged([this]() {
        if (m_syncing) return;
        syncBrushFromView();
    });

    leftLayout->addWidget(m_minimap, 0);

    // Right side: filter list
    auto* rightLayout = new QVBoxLayout;
    rightLayout->setContentsMargins(0, 0, 0, 0);
    auto* filterLabel = new QLabel("Lanes", this);
    QFont f = filterLabel->font(); f.setBold(true); filterLabel->setFont(f);
    rightLayout->addWidget(filterLabel);
    m_filterList = new QListWidget(this);
    m_filterList->setFixedWidth(180);
    rightLayout->addWidget(m_filterList, 1);
    connect(m_filterList, &QListWidget::itemChanged, this, [this](QListWidgetItem*) {
        redraw();
    });

    root->addLayout(leftLayout, 1);
    root->addLayout(rightLayout, 0);

    m_tMin = std::numeric_limits<int>::max();
    m_tMax = std::numeric_limits<int>::min();
}

DesSwimlanePanel::~DesSwimlanePanel() = default;

void DesSwimlanePanel::onInitialize() {
    auto abs = getDisplayContext()->getRosNodeAbstraction().lock();
    if (!abs) return;
    m_node = abs->get_raw_node();

    auto qos = rclcpp::QoS(rclcpp::KeepAll()).reliable().transient_local();

    m_subEvent = m_node->create_subscription<event_system_msgs::msg::TimelineEvent>(
        "/timeline/event", qos,
        [this](const event_system_msgs::msg::TimelineEvent::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() { onEvent(msg); });
        });

    m_subStateChange = m_node->create_subscription<event_system_msgs::msg::TimelineStateChange>(
        "/timeline/state_change", qos,
        [this](const event_system_msgs::msg::TimelineStateChange::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() { onStateChange(msg); });
        });

    m_subMeeting = m_node->create_subscription<event_system_msgs::msg::TimelineMeeting>(
        "/timeline/meeting", qos,
        [this](const event_system_msgs::msg::TimelineMeeting::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() { onMeeting(msg); });
        });

    m_subReset = m_node->create_subscription<event_system_msgs::msg::TimelineReset>(
        "/timeline/reset", qos,
        [this](const event_system_msgs::msg::TimelineReset::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() { onReset(msg); });
        });

    // /system_config is transient_local on the publisher side, so we get the
    // last published value on subscribe and live updates on every config edit.
    m_subConfig = m_node->create_subscription<event_system_msgs::msg::SystemConfig>(
        "/system_config", rclcpp::QoS(1).transient_local(),
        [this](const event_system_msgs::msg::SystemConfig::SharedPtr msg) {
            QMetaObject::invokeMethod(this, [this, msg]() {
                m_lowThreshold = std::clamp(msg->low_battery_threshold / 100.0f, 0.0f, 1.0f);
                scheduleRedraw();
            });
        });
}

void DesSwimlanePanel::onEvent(const event_system_msgs::msg::TimelineEvent::SharedPtr msg) {
    EventRecord r{
        msg->time,
        msg->type,
        QString::fromStdString(msg->label),
        QString::fromStdString(msg->color),
        msg->is_driving,
        msg->is_charging,
        msg->mission_id
    };
    m_events.push_back(std::move(r));
    m_tMin = std::min(m_tMin, msg->time);
    m_tMax = std::max(m_tMax, msg->time);
    scheduleRedraw();
}

void DesSwimlanePanel::onStateChange(const event_system_msgs::msg::TimelineStateChange::SharedPtr msg) {
    m_states.push_back({ msg->time, msg->state, msg->soc });
    m_tMin = std::min(m_tMin, msg->time);
    m_tMax = std::max(m_tMax, msg->time);
    scheduleRedraw();
}

void DesSwimlanePanel::onMeeting(const event_system_msgs::msg::TimelineMeeting::SharedPtr msg) {
    MeetingRecord m{
        msg->id,
        msg->mission_state,
        msg->appointment_time,
        msg->start_time,
        QString::fromStdString(msg->person_name),
        QString::fromStdString(msg->room_name),
        QString::fromStdString(msg->description),
        msg->execution_mode
    };
    m_meetings[msg->id] = std::move(m);
    m_tMin = std::min(m_tMin, msg->start_time);
    m_tMax = std::max(m_tMax, msg->appointment_time);
    scheduleRedraw();
}

void DesSwimlanePanel::onReset(const event_system_msgs::msg::TimelineReset::SharedPtr) {
    m_events.clear();
    m_states.clear();
    m_meetings.clear();
    m_tMin = std::numeric_limits<int>::max();
    m_tMax = std::numeric_limits<int>::min();
    m_brushStart = 0;
    m_brushEnd   = 0;
    m_scene->clear();
    rebuildFilterList();
    if (m_minimap) {
        m_minimap->setSimRange(0, 1);
        m_minimap->setEventTimes({});
    }
}

void DesSwimlanePanel::scheduleRedraw() {
    if (m_redrawScheduled) return;
    m_redrawScheduled = true;
    QTimer::singleShot(50, this, [this]() {
        m_redrawScheduled = false;
        redraw();
    });
}

QColor DesSwimlanePanel::parseColor(const QString& s, const QColor& fallback) {
    if (s.isEmpty()) return fallback;
    QColor c(s);
    return c.isValid() ? c : fallback;
}

QColor DesSwimlanePanel::pickPersonColor(const QString& name) {
    // Try to find a recent event with that person's color
    for (auto it = m_events.rbegin(); it != m_events.rend(); ++it) {
        if (!isPersonEvent(it->type)) continue;
        if (extractPersonName(it->label) == name && !it->color.isEmpty()) {
            return parseColor(it->color, QColor("#888"));
        }
    }
    // Fall back to hash-based color
    uint h = qHash(name);
    return QColor::fromHsv(int(h % 360), 180, 200);
}

QString DesSwimlanePanel::eventTypeName(quint8 type) {
    using TE = event_system_msgs::msg::TimelineEvent;
    switch (type) {
        case TE::SIMULATION_START: return "Simulation Start";
        case TE::SIMULATION_END:   return "Simulation End";
        case TE::STOP_DRIVE:       return "Stop Drive";
        case TE::START_DRIVE:      return "Start Drive";
        case TE::MISSION_DISPATCH: return "Mission Dispatch";
        case TE::MISSION_COMPLETE: return "Mission Complete";
        case TE::START_ACCOMPANY:  return "Start Accompany";
        case TE::ARRIVED_ACCOMPANY: return "Arrived Accompany";
        case TE::START_DROP_OFF_CONV: return "Start Drop-Off Conv.";
        case TE::START_FOUND_PERSON_CONV: return "Start Found-Person Conv.";
        case TE::DROP_OFF_CONV_COMPLETE: return "Drop-Off Conv. Complete";
        case TE::FOUND_PERSON_CONV_COMPLETE: return "Found-Person Conv. Complete";
        case TE::ABORT_SEARCH:     return "Abort Search";
        case TE::BATTERY_FULL:     return "Battery Full";
        case TE::RESET:            return "Reset";
        case TE::PERSON_TRANSITION: return "Person Transition";
        case TE::PERSON_ARRIVED:   return "Person Arrived";
        case TE::PERSON_DEPARTURE: return "Person Departure";
        case TE::PERSON_ACCOMPANY_ARRIVED: return "Person Accompany Arrived";
        case TE::PERSON_ACCOMPANY_DEPARTURE: return "Person Accompany Departure";
        default:                   return QString("Type %1").arg(type);
    }
}

QString DesSwimlanePanel::stateName(quint8 s) {
    using SC = event_system_msgs::msg::TimelineStateChange;
    switch (s) {
        case SC::IDLE:       return "Idle";
        case SC::ACCOMPANY:  return "Accompany";
        case SC::SEARCHING:  return "Searching";
        case SC::CHARGING:   return "Charging";
        case SC::CONVERSATE: return "Conversate";
        case SC::RETURNING:  return "Returning";
        default:             return "Unknown";
    }
}

void DesSwimlanePanel::rebuildFilterList() {
    if (!m_filterList) return;
    // Preserve current checked state
    std::unordered_set<QString> unchecked;
    for (int i = 0; i < m_filterList->count(); ++i) {
        auto* it = m_filterList->item(i);
        if (it->checkState() == Qt::Unchecked) {
            unchecked.insert(it->data(Qt::UserRole).toString());
        }
    }
    m_filterList->blockSignals(true);
    m_filterList->clear();

    auto add = [&](const QString& id, const QString& label) {
        auto* it = new QListWidgetItem(label);
        it->setData(Qt::UserRole, id);
        it->setFlags(it->flags() | Qt::ItemIsUserCheckable);
        it->setCheckState(unchecked.count(id) ? Qt::Unchecked : Qt::Checked);
        m_filterList->addItem(it);
    };
    add(kLaneSystem,  "System");
    // Only show mission lanes for execution modes that actually occurred
    std::unordered_set<quint8> seenModes;
    for (const auto& [id, mr] : m_meetings) seenModes.insert(mr.executionMode);
    // Background missions don't publish a TimelineMeeting — their events arrive
    // with a mission_id that has no matching meeting. Surface them as background.
    for (const auto& e : m_events) {
        if (isMissionEvent(e.type) && e.missionId >= 0
            && m_meetings.find(e.missionId) == m_meetings.end()) {
            seenModes.insert(kExecBackground);
            break;
        }
    }
    // Stable order: Scheduled, Background, Interrupt
    if (seenModes.count(kExecScheduled))  add(kLaneMissionSched, missionLaneLabelFor(kExecScheduled));
    if (seenModes.count(kExecBackground)) add(kLaneMissionBg,    missionLaneLabelFor(kExecBackground));
    if (seenModes.count(kExecInterrupt))  add(kLaneMissionInt,   missionLaneLabelFor(kExecInterrupt));
    // Collect person names
    std::vector<QString> people;
    std::unordered_set<QString> seen;
    for (const auto& e : m_events) {
        if (!isPersonEvent(e.type)) continue;
        QString p = extractPersonName(e.label);
        if (p.isEmpty() || seen.count(p)) continue;
        seen.insert(p);
        people.push_back(p);
    }
    std::sort(people.begin(), people.end());
    for (const auto& p : people) add(kLanePersonPrefix + p, "Person: " + p);
    add(kLaneState,   "Robot State");
    add(kLaneBattery, "Battery");

    m_filterList->blockSignals(false);
}

void DesSwimlanePanel::redraw() {
    if (!m_scene || !m_view) return;

    // Always rebuild filter list (cheap; preserves user state)
    rebuildFilterList();

    auto isLaneVisible = [&](const QString& id) {
        for (int i = 0; i < m_filterList->count(); ++i) {
            auto* it = m_filterList->item(i);
            if (it->data(Qt::UserRole).toString() == id)
                return it->checkState() == Qt::Checked;
        }
        return true;
    };

    m_scene->clear();

    if (m_tMax <= m_tMin) {
        m_scene->setSceneRect(0, 0, 100, 100);
        m_minimap->setSimRange(0, 1);
        m_minimap->setEventTimes({});
        return;
    }

    const int padded_tMin = m_tMin - kPadTime;
    const int padded_tMax = m_tMax + kPadTime;
    const double sceneW = double(padded_tMax - padded_tMin) * kPxPerSecondInit;

    // Persist scene mapping for view↔brush sync (labels are now an external
    // widget, so scene x starts at 0).
    m_paddedTMin    = padded_tMin;
    m_paddedTMax    = padded_tMax;
    m_baseScaleX    = kPxPerSecondInit;
    m_labelColumnPx = 0;

    // Helpers
    auto timeToX = [&](int t) {
        return double(t - padded_tMin) * kPxPerSecondInit;
    };

    // -------- Build lanes layout ---------------------------------------
    struct LaneSlot {
        QString id;
        QString label;
        int yTop;
        int height;
        bool visible;
    };
    std::vector<LaneSlot> lanes;
    int y = kHeaderHeight;

    auto pushLane = [&](const QString& id, const QString& label, int h) {
        lanes.push_back({id, label, y, h, isLaneVisible(id)});
        if (isLaneVisible(id)) y += h;
    };

    pushLane(kLaneSystem,  "System",       kLaneHeightSystem);
    // Mission lanes split by execution mode (only show modes that occurred)
    std::unordered_set<quint8> seenModes;
    for (const auto& [id, mr] : m_meetings) seenModes.insert(mr.executionMode);
    // Background missions don't publish a TimelineMeeting — their events arrive
    // with a mission_id that has no matching meeting. Surface them as background.
    for (const auto& e : m_events) {
        if (isMissionEvent(e.type) && e.missionId >= 0
            && m_meetings.find(e.missionId) == m_meetings.end()) {
            seenModes.insert(kExecBackground);
            break;
        }
    }
    if (seenModes.count(kExecScheduled))
        pushLane(kLaneMissionSched, missionLaneLabelFor(kExecScheduled), kLaneHeightMission);
    if (seenModes.count(kExecBackground))
        pushLane(kLaneMissionBg,    missionLaneLabelFor(kExecBackground), kLaneHeightMission);
    if (seenModes.count(kExecInterrupt))
        pushLane(kLaneMissionInt,   missionLaneLabelFor(kExecInterrupt), kLaneHeightMission);

    // Person lanes (one per person)
    std::vector<QString> people;
    std::unordered_set<QString> seen;
    for (const auto& e : m_events) {
        if (!isPersonEvent(e.type)) continue;
        QString p = extractPersonName(e.label);
        if (p.isEmpty() || seen.count(p)) continue;
        seen.insert(p);
        people.push_back(p);
    }
    std::sort(people.begin(), people.end());
    for (const auto& p : people) {
        pushLane(kLanePersonPrefix + p, "Person: " + p, kLaneHeightPerson);
    }
    pushLane(kLaneState,   "Robot State",  kLaneHeightState);
    pushLane(kLaneBattery, "Battery (SoC)", kBatteryLaneHeight);

    // -------- Rebuild sticky label column ------------------------------
    while (m_labelLayout->count() > 0) {
        auto* it = m_labelLayout->takeAt(0);
        if (auto* w = it->widget()) w->deleteLater();
        delete it;
    }
    auto* headerSpacer = new QWidget;
    headerSpacer->setFixedHeight(kHeaderHeight);
    headerSpacer->setStyleSheet("background:#dce0e8;");
    m_labelLayout->addWidget(headerSpacer);
    int laneRow = 0;
    for (const auto& lane : lanes) {
        if (!lane.visible) continue;
        auto* lbl = new QLabel(lane.label);
        lbl->setFixedHeight(lane.height);
        lbl->setMargin(0);
        lbl->setIndent(10);
        QFont lf = lbl->font();
        lf.setBold(true);
        lbl->setFont(lf);
        QColor bg = laneBg(laneRow++);
        lbl->setStyleSheet(QString("background:%1; color:#283045; "
                                   "border-right:1px solid #b0b6c2;")
                               .arg(bg.name()));
        m_labelLayout->addWidget(lbl);
    }
    m_labelLayout->addStretch(1);

    const int sceneH = y + 10;
    m_scene->setSceneRect(0, 0, sceneW, sceneH);

    // -------- Background bands (labels live in the sticky widget) -------
    int rowIdx = 0;
    for (const auto& lane : lanes) {
        if (!lane.visible) continue;
        auto* bg = m_scene->addRect(
            0, lane.yTop, sceneW, lane.height,
            QPen(Qt::NoPen), QBrush(laneBg(rowIdx++)));
        bg->setZValue(-10);
    }

    // -------- Time axis (header) ---------------------------------------
    auto* axisBg = m_scene->addRect(0, 0, sceneW, kHeaderHeight,
                                    QPen(Qt::NoPen), QBrush(QColor(220, 224, 232)));
    axisBg->setZValue(-10);
    QPen axisPen(QColor(120, 130, 150));
    axisPen.setCosmetic(true);
    QPen gridPen(QColor(210, 215, 225));
    gridPen.setCosmetic(true);
    gridPen.setStyle(Qt::DashLine);

    // Tick every hour
    constexpr int tickStep = 3600;
    int first = (padded_tMin / tickStep) * tickStep;
    if (first < padded_tMin) first += tickStep;
    QFont tickFont = m_view->font();
    tickFont.setPointSize(std::max(7, tickFont.pointSize() - 1));
    for (int t = first; t <= padded_tMax; t += tickStep) {
        double x = timeToX(t);
        auto* line = m_scene->addLine(x, 0, x, sceneH, gridPen);
        line->setZValue(-5);
        auto* tickLine = m_scene->addLine(x, 0, x, kHeaderHeight, axisPen);
        tickLine->setZValue(0);
        auto* txt = m_scene->addSimpleText(humanTime(t), tickFont);
        txt->setBrush(QColor(60, 60, 90));
        txt->setPos(x + 2, 2);
        txt->setZValue(5);
    }

    // -------- Lane lookup by id ----------------------------------------
    auto laneOf = [&](const QString& id) -> const LaneSlot* {
        for (const auto& l : lanes) if (l.id == id) return &l;
        return nullptr;
    };

    // -------- Mission spans --------------------------------------------
    // Span = MISSION_START → MISSION_COMPLETE (the active phase). MISSION_DISPATCH
    // stays as a small marker before the span (Pending → Active transition is the
    // span's left edge). If a mission is rejected directly after dispatch, there
    // is no MISSION_START and therefore no span — only Dispatch + Complete markers.
    {
        struct Open { int startTime; QString label; };
        std::map<int, Open> open;
        auto extractId = [](const QString& label) -> int {
            QStringList parts = label.split(' ');
            if (parts.size() < 2 || parts[0] != "Mission") return -1;
            bool ok = false;
            int id = parts[1].toInt(&ok);
            return ok ? id : -1;
        };
        // Final state parsed from MISSION_COMPLETE label: "Mission 5 Complete: Rejected"
        auto stateFromCompleteLabel = [](const QString& label) -> QString {
            int colon = label.indexOf(':');
            if (colon < 0) return {};
            return label.mid(colon + 1).trimmed();
        };
        auto spanColorForState = [](const QString& state) -> QColor {
            if (state.compare("Completed",   Qt::CaseInsensitive) == 0) return QColor("#1976d2");
            if (state.compare("Rejected",    Qt::CaseInsensitive) == 0) return QColor("#9e9e9e");
            if (state.compare("Failed",      Qt::CaseInsensitive) == 0) return QColor("#d32f2f");
            if (state.compare("Cancelled",   Qt::CaseInsensitive) == 0) return QColor("#bdbdbd");
            return QColor("#1976d2");
        };
        auto laneForId = [&](int id) -> const LaneSlot* {
            quint8 mode = kExecBackground;
            if (auto it = m_meetings.find(id); it != m_meetings.end())
                mode = it->second.executionMode;
            return laneOf(missionLaneIdFor(mode));
        };
        for (const auto& e : m_events) {
            using TE = event_system_msgs::msg::TimelineEvent;
            int id = e.missionId;
            if (id < 0) id = extractId(e.label);
            if (id < 0) continue;

            // BG/SCHED missions become active at MISSION_START (fired by the
            // BT's AcceptMissionAction). Interrupts have no MISSION_START —
            // they become active directly at ORDER_ARRIVAL.
            bool spanStart = (e.type == TE::MISSION_START);
            if (e.type == TE::ORDER_ARRIVAL) {
                if (auto it = m_meetings.find(id);
                    it != m_meetings.end() && it->second.executionMode == kExecInterrupt) {
                    spanStart = true;
                }
            }
            if (spanStart) {
                if (open.find(id) == open.end()) open[id] = { e.time, e.label };
            } else if (e.type == TE::MISSION_COMPLETE) {
                auto it = open.find(id);
                if (it == open.end()) continue;  // no span (e.g. rejected w/o start)
                const LaneSlot* lane = laneForId(id);
                if (!lane || !lane->visible) { open.erase(it); continue; }
                const QString stateStr = stateFromCompleteLabel(e.label);
                const QColor c = spanColorForState(stateStr);
                const int t0 = it->second.startTime;
                const int t1 = e.time;
                const double x0 = timeToX(t0);
                const double x1 = timeToX(t1);
                const int spanY = lane->yTop + (lane->height - kSpanHeight) / 2;
                QPen pen(c.darker(140));
                pen.setCosmetic(true);
                auto* rect = m_scene->addRect(x0, spanY, std::max(2.0, x1 - x0), kSpanHeight,
                                              pen, QBrush(c.lighter(140)));
                rect->setZValue(2);
                QString tooltip = QString("<b>Mission %1</b><br>").arg(id);

                // Punctuality: for scheduled missions with a real deadline,
                // draw a dashed vertical line at the deadline and a thin
                // diff bar (green=early, red=late) below the span.
                int deadline = -1;
                quint8 mode = kExecBackground;
                if (auto mIt = m_meetings.find(id); mIt != m_meetings.end()) {
                    mode = mIt->second.executionMode;
                    const char* modeLabel =
                        (mode == kExecScheduled) ? "Scheduled" :
                        (mode == kExecInterrupt) ? "Interrupt" : "Background";
                    tooltip += QString("Mode: %1<br>").arg(modeLabel);
                    if (!mIt->second.personName.isEmpty() || !mIt->second.roomName.isEmpty()) {
                        tooltip += QString("Person: %1<br>Room: %2<br>")
                                       .arg(mIt->second.personName, mIt->second.roomName);
                    }
                    // Real deadline = appointmentTime distinct from startTime
                    if (mIt->second.appointmentTime > 0
                        && mIt->second.appointmentTime != mIt->second.startTime) {
                        deadline = mIt->second.appointmentTime;
                    }
                }
                if (!stateStr.isEmpty()) tooltip += QString("State: %1<br>").arg(stateStr);
                tooltip += QString("Start: %1<br>End: %2<br>Duration: %3 s")
                               .arg(humanTime(t0), humanTime(t1)).arg(t1 - t0);

                if (mode == kExecScheduled && deadline > 0) {
                    const double xD = timeToX(deadline);
                    QPen dlPen(QColor(50, 50, 60, 220));
                    dlPen.setStyle(Qt::DashLine);
                    dlPen.setCosmetic(true);
                    auto* dlLine = m_scene->addLine(xD, lane->yTop + 2,
                                                    xD, lane->yTop + lane->height - 2, dlPen);
                    dlLine->setZValue(3);
                    dlLine->setToolTip(QString("<b>Mission %1 deadline</b><br>%2")
                                           .arg(id).arg(humanTime(deadline)));

                    const int diff = t1 - deadline;  // positive = late, negative = early
                    if (diff != 0) {
                        const QColor diffColor = (diff < 0) ? QColor("#2e7d32")   // green
                                                            : QColor("#c62828");  // red
                        const double xA = std::min(timeToX(t1), xD);
                        const double xB = std::max(timeToX(t1), xD);
                        const int diffY  = spanY + kSpanHeight + 1;
                        constexpr int diffH = 4;
                        auto* diffBar = m_scene->addRect(xA, diffY, xB - xA, diffH,
                                                         QPen(Qt::NoPen), QBrush(diffColor));
                        diffBar->setZValue(2);
                        const QString diffLbl = (diff < 0)
                            ? QString("%1 s early").arg(-diff)
                            : QString("%1 s late").arg(diff);
                        diffBar->setToolTip(QString("<b>Mission %1</b><br>%2").arg(id).arg(diffLbl));
                        tooltip += QString("<br>Deadline: %1<br>%2")
                                       .arg(humanTime(deadline), diffLbl);
                    } else {
                        tooltip += QString("<br>Deadline: %1<br>on time").arg(humanTime(deadline));
                    }
                }

                rect->setToolTip(tooltip);
                open.erase(it);
            }
        }
        // Still-open spans (mission started, no Complete seen yet) up to m_tMax
        for (const auto& [id, op] : open) {
            const LaneSlot* lane = laneForId(id);
            if (!lane || !lane->visible) continue;
            const double x0 = timeToX(op.startTime);
            const double x1 = timeToX(m_tMax);
            const int spanY = lane->yTop + (lane->height - kSpanHeight) / 2;
            QPen pen(QColor(160, 160, 160));
            pen.setCosmetic(true);
            pen.setStyle(Qt::DashLine);
            auto* rect = m_scene->addRect(x0, spanY, std::max(2.0, x1 - x0), kSpanHeight,
                                          pen, QBrush(QColor(220, 220, 220, 160)));
            rect->setZValue(2);
            rect->setToolTip(QString("<b>Mission %1 (running)</b><br>Started: %2")
                                 .arg(id).arg(humanTime(op.startTime)));
        }
    }

    // -------- Event markers as type-specific glyphs --------------------
    auto drawEvent = [&](int t, const LaneSlot& lane, quint8 type,
                         const QColor& c, const QString& tooltip) {
        const double x = timeToX(t);
        const double y = lane.yTop + lane.height * subBandFor(type);
        const GlyphSpec g = glyphFor(type);
        QGraphicsItem* item = makeGlyphItem(m_scene, g, c, c.darker(170));
        if (!item) return;
        item->setPos(x, y);
        item->setZValue(4);
        item->setToolTip(tooltip);
    };

    for (const auto& e : m_events) {
        QString tooltip = QString(
            "<b>%1</b><br>"
            "Time: %2<br>"
            "%3"
        ).arg(eventTypeName(e.type), humanTime(e.time), e.label);

        if (isSystemEvent(e.type)) {
            if (auto* lane = laneOf(kLaneSystem); lane && lane->visible)
                drawEvent(e.time, *lane, e.type,
                          parseColor(e.color, QColor("#333")), tooltip);
        } else if (isMissionEvent(e.type)) {
            // Route to lane by mission ID → execution mode.
            // Prefer the explicit field (populated by Context from m_current);
            // fall back to label parsing for older msgs without mission_id.
            int missionId = e.missionId;
            if (missionId < 0) {
                QStringList parts = e.label.split(' ');
                if (parts.size() >= 2 && parts[0] == "Mission") {
                    bool ok = false;
                    int id = parts[1].toInt(&ok);
                    if (ok) missionId = id;
                }
            }
            quint8 mode = kExecBackground;
            if (missionId >= 0) {
                if (auto it = m_meetings.find(missionId); it != m_meetings.end())
                    mode = it->second.executionMode;
            }
            if (auto* lane = laneOf(missionLaneIdFor(mode)); lane && lane->visible) {
                // Colors aligned with timeline_types.hpp::getEventColor
                QColor base;
                using TE = event_system_msgs::msg::TimelineEvent;
                switch (e.type) {
                    case TE::MISSION_DISPATCH:
                    case TE::MISSION_COMPLETE:
                        base = QColor(220, 0, 0); break;
                    case TE::ABORT_SEARCH:
                        base = QColor(255, 165, 0); break;
                    case TE::START_DROP_OFF_CONV:
                    case TE::START_FOUND_PERSON_CONV:
                    case TE::DROP_OFF_CONV_COMPLETE:
                    case TE::FOUND_PERSON_CONV_COMPLETE:
                        base = QColor(130, 0, 200); break;
                    case TE::START_ACCOMPANY:
                    case TE::ARRIVED_ACCOMPANY:
                        base = QColor(0, 100, 255); break;
                    case TE::START_DRIVE:
                    case TE::STOP_DRIVE:
                        base = QColor(0, 150, 50); break;
                    case TE::MISSION_START:
                        base = QColor(220, 0, 0); break;
                    default:
                        base = Qt::black;
                }
                drawEvent(e.time, *lane, e.type, parseColor(e.color, base), tooltip);
            }
        } else if (isPersonEvent(e.type)) {
            QString p = extractPersonName(e.label);
            if (auto* lane = laneOf(kLanePersonPrefix + p); lane && lane->visible) {
                drawEvent(e.time, *lane, e.type,
                          parseColor(e.color, pickPersonColor(p)), tooltip);
            }
        }
    }

    // -------- Robot state segments -------------------------------------
    if (auto* lane = laneOf(kLaneState); lane && lane->visible && !m_states.empty()) {
        auto sorted = m_states;
        // stable_sort preserves arrival order between state-changes with the
        // same timestamp (e.g. BT-tick + event-execute in the same step).
        std::stable_sort(sorted.begin(), sorted.end(),
                         [](const auto& a, const auto& b){ return a.time < b.time; });
        for (size_t i = 0; i < sorted.size(); ++i) {
            int t0 = sorted[i].time;
            int t1 = (i + 1 < sorted.size()) ? sorted[i + 1].time : m_tMax;
            if (t1 <= t0) continue;
            double x0 = timeToX(t0);
            double x1 = timeToX(t1);
            QColor c = stateColor(sorted[i].state);
            QPen pen(c.darker(140));
            pen.setCosmetic(true);
            auto* seg = m_scene->addRect(x0, lane->yTop + 4, std::max(2.0, x1 - x0), lane->height - 8,
                                         pen, QBrush(c));
            seg->setZValue(2);
            seg->setToolTip(QString("<b>%1</b><br>%2 → %3")
                                .arg(stateName(sorted[i].state), humanTime(t0), humanTime(t1)));
        }
    }

    // -------- Battery curve --------------------------------------------
    if (auto* lane = laneOf(kLaneBattery); lane && lane->visible && !m_states.empty()) {
        auto sorted = m_states;
        std::sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b){ return a.time < b.time; });

        // Threshold (0..1) comes from TimelineStateChange.low_threshold,
        // populated in onStateChange.
        const float lowThr = m_lowThreshold;

        const double topY    = lane->yTop + 6;
        const double botY    = lane->yTop + lane->height - 4;
        const double bandH   = botY - topY;
        auto socToY = [&](float soc) {
            float v = std::clamp(soc, 0.0f, 1.0f);
            return botY - v * bandH;
        };
        // Low-battery band
        {
            QBrush br(QColor(255, 80, 80, 60));
            auto* band = m_scene->addRect(0, botY - lowThr * bandH,
                                          sceneW, lowThr * bandH,
                                          QPen(Qt::NoPen), br);
            band->setZValue(0);
        }

        // Path
        QPainterPath path;
        path.moveTo(timeToX(sorted.front().time), socToY(sorted.front().soc));
        for (size_t i = 1; i < sorted.size(); ++i) {
            path.lineTo(timeToX(sorted[i].time), socToY(sorted[i].soc));
        }
        QPen p(QColor(30, 30, 30));
        p.setWidth(2);
        p.setCosmetic(true);
        auto* line = m_scene->addPath(path, p);
        line->setZValue(4);

        // Hover dots (sticky size regardless of zoom)
        for (const auto& s : sorted) {
            auto* dot = m_scene->addEllipse(-3, -3, 6, 6,
                                            QPen(QColor(30, 30, 30)),
                                            QBrush(QColor(30, 30, 30)));
            dot->setPos(timeToX(s.time), socToY(s.soc));
            dot->setFlag(QGraphicsItem::ItemIgnoresTransformations, true);
            dot->setZValue(5);
            dot->setToolTip(QString("<b>SoC %1%</b><br>%2")
                                .arg(int(s.soc * 100)).arg(humanTime(s.time)));
        }
    }

    m_view->setSceneRect(m_scene->sceneRect());

    // -------- Minimap update -------------------------------------------
    std::vector<int> evTimes;
    evTimes.reserve(m_events.size());
    for (const auto& e : m_events) evTimes.push_back(e.time);

    m_syncing = true;
    m_minimap->setSimRange(padded_tMin, padded_tMax);
    m_minimap->setEventTimes(evTimes);

    bool firstView = (m_brushEnd <= m_brushStart);
    if (firstView) {
        // Fit-to-window: scale view so the full sim range fits the viewport
        const double viewportPx = std::max(50, m_view->viewport()->width() - kLabelColumnWidth);
        const double scaleX = viewportPx / std::max(1.0, sceneW - kLabelColumnWidth);
        QTransform t;
        t.scale(scaleX, 1.0);
        m_view->setTransform(t);
        m_view->horizontalScrollBar()->setValue(0);
        m_brushStart = padded_tMin;
        m_brushEnd   = padded_tMax;
    } else {
        m_brushStart = std::clamp(m_brushStart, padded_tMin, padded_tMax);
        m_brushEnd   = std::clamp(m_brushEnd,   padded_tMin, padded_tMax);
    }
    m_minimap->setBrush(m_brushStart, m_brushEnd);
    m_syncing = false;

    // Now that scene is laid out, push current viewport into brush
    syncBrushFromView();
}

void DesSwimlanePanel::syncBrushFromView() {
    if (m_syncing || !m_view || !m_minimap) return;
    if (m_paddedTMax <= m_paddedTMin) return;
    // Map viewport edges to scene x, then to time
    const int vpW = m_view->viewport()->width();
    QPointF leftScene  = m_view->mapToScene(0, 0);
    QPointF rightScene = m_view->mapToScene(vpW, 0);
    auto xToTime = [&](double x) {
        double seconds = (x - m_labelColumnPx) / m_baseScaleX;
        return m_paddedTMin + int(std::round(seconds));
    };
    int t0 = std::clamp(xToTime(leftScene.x()),  m_paddedTMin, m_paddedTMax);
    int t1 = std::clamp(xToTime(rightScene.x()), m_paddedTMin, m_paddedTMax);
    if (t1 <= t0) t1 = t0 + 1;
    m_brushStart = t0;
    m_brushEnd   = t1;
    m_syncing = true;
    m_minimap->setBrush(t0, t1);
    m_syncing = false;
}

void DesSwimlanePanel::syncViewFromBrush() {
    if (m_syncing || !m_view) return;
    if (m_brushEnd <= m_brushStart) return;
    const double targetSeconds = double(m_brushEnd - m_brushStart);
    const double viewportPx    = std::max(50, m_view->viewport()->width() - m_labelColumnPx);
    const double scaleX        = viewportPx / (targetSeconds * m_baseScaleX);
    m_syncing = true;
    QTransform t;
    t.scale(scaleX, 1.0);
    m_view->setTransform(t);
    // Scroll so brushStart-in-scene-x is at viewport's left edge.
    const double sceneX = m_labelColumnPx + double(m_brushStart - m_paddedTMin) * m_baseScaleX;
    const int targetScroll = int(sceneX * scaleX);
    m_view->horizontalScrollBar()->setValue(targetScroll);
    m_syncing = false;
}

}  // namespace des_swimlane_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(des_swimlane_panel::DesSwimlanePanel, rviz_common::Panel)
