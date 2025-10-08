#include <cmath>
#include <QApplication>
#include <queue>
#include <random>

#include <QtCharts/QChartView>
#include <QtCharts/QBarSeries>
#include <QtCharts/QBarSet>
#include <QtCharts/QBarCategoryAxis>
#include <QtCharts/QValueAxis>

#include "../include/rnd.h"

int distTest(int argc, char *argv[]) {
    QGraphicsView view{};
    QGraphicsScene scene{};

    //std::vector<double> data = normalDist(10, 5, 1000);
    std::vector<double> data = rnd::dist(std::exponential_distribution<>(), 1000);

    const int numBins = 20;
    std::map<int, int> histogram;

    double minVal = *std::min_element(data.begin(), data.end());
    double maxVal = *std::max_element(data.begin(), data.end());
    double binWidth = (maxVal - minVal) / numBins;

    for (double val : data) {
        int bin = std::min((int)((val - minVal) / binWidth), numBins - 1);
        histogram[bin]++;
    }

    QtCharts::QBarSet *set = new QtCharts::QBarSet("Häufigkeit");
    QStringList categories;
    for (int i = 0; i < numBins; i++) {
        *set << histogram[i];
        double binCenter = minVal + (i + 0.5) * binWidth;
        categories << QString::number(binCenter, 'f', 1);
    }

    QtCharts::QBarSeries *series = new QtCharts::QBarSeries();
    series->append(set);

    QtCharts::QChart *chart = new QtCharts::QChart();
    chart->addSeries(series);
    chart->setTitle("Normalverteilung Histogramm");

    QtCharts::QBarCategoryAxis *axisX = new QtCharts::QBarCategoryAxis();
    axisX->append(categories);
    axisX->setTitleText("Wert");
    chart->addAxis(axisX, Qt::AlignBottom);
    series->attachAxis(axisX);

    QtCharts::QValueAxis *axisY = new QtCharts::QValueAxis();
    axisY->setTitleText("Histogram");
    chart->addAxis(axisY, Qt::AlignLeft);
    series->attachAxis(axisY);

    QtCharts::QChartView *chartView = new QtCharts::QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);

    chartView->setSceneRect(100, 100, 600,600);
    chartView->show();
    return QApplication::exec();
}
