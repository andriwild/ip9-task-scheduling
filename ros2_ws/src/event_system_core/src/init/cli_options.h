#pragma once

#include <QCommandLineParser>
#include <QCoreApplication>
#include <string>

inline std::string DEFAULT_SIM_CONFIG = "sim_config.json";
inline std::string DEFAULT_APPTS      = "appointments.json";


struct CliOptions {
    bool stepMode;
    bool headless;
    int delayMs;
    std::string simConfigPath;
    std::string appointmentConfigPath;
};

inline CliOptions parseCliArguments(const QCoreApplication& app) {
    QCommandLineParser parser;
    parser.setApplicationDescription("Discrete Event System Simulation");
    parser.addHelpOption();
    parser.addVersionOption();

    QCommandLineOption robotConfigOption(QStringList() << "r" << "robot-config", "Path to config file.", "file");
    parser.addOption(robotConfigOption);
    
    QCommandLineOption appointmentConfigOption(QStringList() << "a" << "appointment-config", "Path to config file.", "file");
    parser.addOption(appointmentConfigOption);

    QCommandLineOption stepOption(QStringList() << "s" << "step", "Manual step mode.");
    parser.addOption(stepOption);

    QCommandLineOption timeoutOption(QStringList() << "t" << "timeout", "Delay in ms.", "ms", "0");
    parser.addOption(timeoutOption);

    QCommandLineOption headlessOption(QStringList() << "H" << "headless", "Headless run.");
    parser.addOption(headlessOption);

    parser.process(app);

    const QStringList args = parser.positionalArguments();

    std::string rConfig = DEFAULT_SIM_CONFIG;
    if (parser.isSet(robotConfigOption)) rConfig = parser.value(robotConfigOption).toStdString();
    else if (!args.isEmpty()) rConfig = args.at(0).toStdString();

    std::string aConfig = DEFAULT_APPTS;
    if (parser.isSet(appointmentConfigOption)) aConfig = parser.value(appointmentConfigOption).toStdString();
    else if (args.size() > 1) aConfig = args.at(1).toStdString();

    return {
        parser.isSet(stepOption),
        parser.isSet(headlessOption),
        parser.value(timeoutOption).toInt(),
        rConfig,
        aConfig
    };
}
