#pragma once
#include <qcolor.h>

enum class TaskType { CHARGE, DRIVE, ESCORT };

inline QColor taskColor(const TaskType type) {
   switch (type) {
      case TaskType::CHARGE: return {255, 165, 0};
      case TaskType::DRIVE:  return Qt::gray;
      case TaskType::ESCORT: return Qt::red;
   }
   Q_UNREACHABLE();
}

struct Task {
   int startTime;
   int duration;
   TaskType type;
};