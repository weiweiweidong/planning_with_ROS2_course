#include "local_trajectory_combiner.h"

namespace Planning
{
    LocalTrajectoryCombiner::LocalTrajectoryCombiner() // 轨迹合成器
    {
        RCLCPP_INFO(rclcpp::get_logger("local_trajectory"), "local_trajectory_combiner created");

        // 读取配置文件 (因为没有填写配置参数，实际上在这一部分中用不到。为了维护代码风格的统一，就添加了读取的操作)
        trajectory_config_ = std::make_unique<ConfigReader>();
    }

    // 合成轨迹
    LocalTrajectory LocalTrajectoryCombiner::combin_trajectory(const LocalPath &path, const LocalSpeeds &speeds)
    {
        const int path_size = path.local_path.size();
        const int speeds_size = speeds.local_speeds.size();
        local_trajectory_.header = path.header;
        local_trajectory_.local_trajectory.clear();

        if (path_size < 3 || speeds_size < 3) // 少于3个点，就视为空
        {
            RCLCPP_WARN(rclcpp::get_logger("trajectory"), "local path or speed empty!");
            return local_trajectory_;
        }

        LocalTrajectoryPoint point_tmp;
        for (int i = 0; i < path_size; i++)
        {
            // 路径部分的填充
            point_tmp.path_point = path.local_path[i]; // 真实路径第i个点的数据，全部打包拷贝到临时点里面的路径点

            // 速度部分的赋值
            if (i < speeds_size) // 防止越界。因为路径长度是随速度动态变化的，所以可以直接用速度规划的下标赋值
            {
                point_tmp.speed_point = speeds.local_speeds[i];
            }

            // 填充进轨迹
            local_trajectory_.local_trajectory.emplace_back(point_tmp);
        }

        RCLCPP_INFO(rclcpp::get_logger("tragectory"), "trajectory combined, size = %ld", local_trajectory_.local_trajectory.size());
        return local_trajectory_;
    }

} // namespace Planning