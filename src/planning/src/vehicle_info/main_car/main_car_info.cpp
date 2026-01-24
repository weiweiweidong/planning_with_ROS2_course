#include "main_car_info.h"

namespace Planning
{
    MainCar::MainCar() // 主车信息
    {
        RCLCPP_INFO(rclcpp::get_logger("vehicle"), "main_car created");

        // 读取配置
        vehicle_config_ = std::make_unique<ConfigReader>();
        vehicle_config_->read_vehicles_config();

        // 更新基本属性
        child_frame_ = vehicle_config_->main_car().frame_;
        length_ = vehicle_config_->main_car().length_;
        width_ = vehicle_config_->main_car().width_;
        theta_ = vehicle_config_->main_car().pose_theta_;
        speed_ = vehicle_config_->main_car().speed_ori_;
        id_ = 0;

        // 初始化定位点
        tf2::Quaternion qtn;          // 四元数：在ros2中表达一个车辆在某一点的状态，xyz表示位置坐标，w表示旋转角度
        qtn.setRPY(0.0, 0.0, theta_); // 三个参数是依次绕xyz方向的转角。
        loc_point_.header.frame_id = vehicle_config_->pnc_map().frame_;
        loc_point_.header.stamp = rclcpp::Clock().now();
        loc_point_.pose.position.x = vehicle_config_->main_car().pose_x_;
        loc_point_.pose.position.y = vehicle_config_->main_car().pose_y_;
        loc_point_.pose.position.z = 0;
        loc_point_.pose.orientation.x = qtn.getX();
        loc_point_.pose.orientation.y = qtn.getY();
        loc_point_.pose.orientation.z = qtn.getZ();
        loc_point_.pose.orientation.w = qtn.getW();
    }
} // namespace Planning