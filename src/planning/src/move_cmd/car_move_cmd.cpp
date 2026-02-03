#include "car_move_cmd.h"

namespace Planning
{
    CarMoveCmd::CarMoveCmd() : Node("car_move_cmd_node") // 主车运动指令
    {
        RCLCPP_INFO(this->get_logger(), "car_move_cmd_node created");

        // 读取配置文件，获取父子坐标
        move_cmd_config_ = std::make_unique<ConfigReader>();
        move_cmd_config_->read_move_cmd_config(); // 虽然运动模块本身没有配参数，但是点进去后会发现运动模块本身会读取地图的坐标系。所以还是要写这一句

        // 初始化
        car_ = std::make_shared<MainCar>();
        car_param_.pos_x_ = car_->loc_point().pose.position.x;
        car_param_.pos_y_ = car_->loc_point().pose.position.y;
        car_param_.theta_ = car_->theta();
        car_param_.speed_ = car_->speed();

        // 创建坐标广播方
        broadcaster_ = std::make_shared<TransformBroadcaster>(this);

        // 创建轨迹订阅器，广播运动指令
        local_trajectory_sub_ = this->create_subscription<LocalTrajectory>(
            "planning/local_trajectory",
            10,
            std::bind(&CarMoveCmd::car_broadcast_tf, this, _1));
    }

    // 广播主车坐标变换
    void CarMoveCmd::car_broadcast_tf(const LocalTrajectory::SharedPtr trajectory)
    {
        (void)trajectory;
    }
} // namespace Planning

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planning::CarMoveCmd>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}