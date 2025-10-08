#pragma once
#include <qcolor.h>

enum class TaskType { CHARGE, DRIVE, ESCORT, INSPECTION, COMPUTE };

inline QColor taskColor(const TaskType type) {
   switch (type) {
      case TaskType::CHARGE:     return {255, 165, 0}; // orange
      case TaskType::DRIVE:      return Qt::gray;
      case TaskType::ESCORT:     return Qt::red;
      case TaskType::INSPECTION: return Qt::blue;
      case TaskType::COMPUTE:    return Qt::green;
   }
   Q_UNREACHABLE();
}

struct task {
   int startTime;
   int duration;
   TaskType type;
};