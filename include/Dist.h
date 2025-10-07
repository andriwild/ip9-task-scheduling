#pragma once

#include <cmath>
#include <QApplication>
#include <random>

#include <QtCharts/QChartView>
#include <QtCharts/QBarSeries>
#include <QtCharts/QBarSet>
#include <QtCharts/QBarCategoryAxis>
#include <QtCharts/QValueAxis>

namespace dist {
    static std::random_device rd{};
    static std::mt19937 gen{rd()};

    inline double uniRand() {
        int r;
        do {
            r = rand();
        } while (r == 0);
        return static_cast<double>(r) / RAND_MAX;
    }

    inline double expRand(double mean) {
        return -mean * std::log(uniRand());
    }

    inline double expRand2(double rate) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::exponential_distribution d(rate);
        return d(gen);
    }

    inline std::vector<double> expDist2(double mean, int n) {
        std::vector<double> values(n);
        std::ranges::generate(values, [&] {return expRand(mean);});
        return values;
    }

    template<typename Distribution>
    std::vector<double> dist(Distribution d, const int n) {
        std::vector<double> values(n);
        std::ranges::generate(values, [&] {return d(gen);});
        return values;
    }

    template<typename Distribution>
    double rnd(Distribution d) {
        return d(gen);
    }


    inline int show() {
        QGraphicsView view{};
        QGraphicsScene scene{};
        constexpr int n = 1000;

        //std::vector<double> data = dist(std::exponential_distribution<>(), n);
        //std::vector<double> data = dist(std::normal_distribution<>(10, 4), n);
        std::vector<double> data = dist(std::poisson_distribution<>(20), n);


        constexpr int numBins = 50;
        std::map<int, int> histogram;

        const double minVal = *std::ranges::min_element(data);
        const double maxVal = *std::ranges::max_element(data);
        const double binWidth = (maxVal - minVal) / numBins;

        for (const double val : data) {
            int bin = std::min((int)((val - minVal) / binWidth), numBins - 1);
            histogram[bin]++;
        }

        auto set = new QtCharts::QBarSet("Häufigkeit");
        QStringList categories;
        for (int i = 0; i < numBins; i++) {
            *set << histogram[i];
            double binCenter = minVal + (i + 0.5) * binWidth;
            categories << QString::number(binCenter, 'f', 1);
        }

        auto series = new QtCharts::QBarSeries();
        series->append(set);

        auto chart = new QtCharts::QChart();
        chart->addSeries(series);
        chart->setTitle("Histogramm");

        auto axisX = new QtCharts::QBarCategoryAxis();
        axisX->append(categories);
        axisX->setTitleText("Wert");
        chart->addAxis(axisX, Qt::AlignBottom);
        series->attachAxis(axisX);

        auto axisY = new QtCharts::QValueAxis();
        axisY->setTitleText("Anzahl");
        chart->addAxis(axisY, Qt::AlignLeft);
        series->attachAxis(axisY);

        auto chartView = new QtCharts::QChartView(chart);
        chartView->setRenderHint(QPainter::Antialiasing);

        chartView->setSceneRect(100, 100, 800,600);
        chartView->show();
        return QApplication::exec();
    }

}
