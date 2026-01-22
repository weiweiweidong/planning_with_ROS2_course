#ifndef CURVE_H_
#define CURVE_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/referline.hpp"
#include "base_msgs/msg/local_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>

namespace Planning
{
    using base_msgs::msg::LocalPath;
    using base_msgs::msg::Referline;
    using geometry_msgs::msg::PoseStamped;
    using nav_msgs::msg::Path;

    class Curve // 曲线
    {
    public:
        Curve() = default;

        // 找匹配点下标
        static int find_match_point(const Path &path, const int &last_match_point_index, const PoseStamped &target_point);
    };
} // namespace Planning
#endif // CURVE_H_