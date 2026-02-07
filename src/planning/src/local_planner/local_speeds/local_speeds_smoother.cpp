#include "local_speeds_smoother.h"

namespace Planning
{
    LocalSpeedsSmoother::LocalSpeedsSmoother() // 速度平滑器
    {
        RCLCPP_INFO(rclcpp::get_logger("local_speed"), "local_speeds_smoother created");

        // 读取配置文件
        local_speeds_config_ = std::make_unique<ConfigReader>();
        local_speeds_config_->read_local_path_config();
    }

    // 平滑速度
    void LocalSpeedsSmoother::smooth_local_speeds(LocalSpeeds speeds)
    {
        RCLCPP_INFO(rclcpp::get_logger("local_speed"), "local speeds smoothed");

        (void)speeds;
    }
} // namespace Planning