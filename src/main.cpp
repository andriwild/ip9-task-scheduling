#include <QApplication>

#include "../include/MainView.h"
#include "../include/Timeline.h"

constexpr int VIEW_HEIGHT = 500;
constexpr int SCENE_MARGIN = 50;

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    QApplication::setFont({"Arial"});
    MainView view{};
    QGraphicsScene scene{};

    constexpr int timelineEnd = 3600 * 12;
    scene.setSceneRect(-SCENE_MARGIN, 0, timelineEnd + SCENE_MARGIN, VIEW_HEIGHT);

    const Timeline timeline {&scene, timelineEnd};
    timeline.draw();

    std::vector<Task> const tasks = {
        { 100, 200, TaskType::DRIVE},
        { 300, 400, TaskType::ESCORT},
        { 700, 200, TaskType::DRIVE},
        {1100, 300, TaskType::CHARGE}
    };
    timeline.drawTasks(tasks);

    view.setScene(&scene);
    view.setDragMode(QGraphicsView::ScrollHandDrag);
    view.setRenderHint(QPainter::Antialiasing);
    view.centerOn(0,0);
    view.show();
    return QApplication::exec();
}