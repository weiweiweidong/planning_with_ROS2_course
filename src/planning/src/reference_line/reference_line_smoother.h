#ifndef REFERENCE_LINE_SMOOTHER_H_
#define REFERENCE_LINE_SMOOTHER_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <cmath>

namespace Planning
{
    class ReferenceLineSmoother  // 参考线平滑
    {
        public:
            ReferenceLineSmoother();
        private:
    };
}  // namespace Planning
#endif  // REFERENCE_LINE_SMOOTHER_H_
