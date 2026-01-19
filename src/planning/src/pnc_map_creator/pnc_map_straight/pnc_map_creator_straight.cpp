#include "pnc_map_creator_straight.h"

namespace Planning
{
    PNCMapCreatorStraight::PNCMapCreatorStraight() // 直道地图
    {
        RCLCPP_INFO(rclcpp::get_logger("pnc_map"), "pnc_map_creator created");

        // 读取配置文件，给参数赋值
        pnc_map_config_ = std::make_unique<ConfigReader>();
        pnc_map_config_->read_pnc_map_config();
        map_type_ = static_cast<int>(PNCMapType::STRAIGHT);

        // 地图起点坐标
        p_mid_.x = -3.0;
        p_mid_.y = pnc_map_config_->pnc_map().road_half_width_ / 2.0;

        // 长度步长
        len_step_ = pnc_map_config_->pnc_map().segment_len_;

        // 地图初始化
        init_pnc_map();
    }

    PNCMap PNCMapCreatorStraight::create_pnc_map() // 生成地图
    {
        return pnc_map_;
    }

    void PNCMapCreatorStraight::init_pnc_map()
    {
        pnc_map_.header.frame_id = pnc_map_config_->pnc_map().frame_;
        pnc_map_.header.stamp = rclcpp::Clock().now();
        pnc_map_.road_length = pnc_map_config_->pnc_map().road_length_;
        pnc_map_.road_half_width = pnc_map_config_->pnc_map().road_half_width_;

        // 中心线格式
        pnc_map_.midline.header = pnc_map_.header;
        pnc_map_.midline.ns = "pnc_map";
        pnc_map_.midline.id = 0;
        pnc_map_.midline.action = Marker::ADD;
        pnc_map_.midline.type = Marker::LINE_LIST;           // 分段线条
        pnc_map_.midline.scale.x = 0.05;                     // 线段宽度
        pnc_map_.midline.color.a = 1.0;                      // 透明度：不透明
        pnc_map_.midline.color.r = 0.7;                      // 红色
        pnc_map_.midline.color.g = 0.7;                      // 绿色
        pnc_map_.midline.color.b = 0.0;                      // 蓝色
        pnc_map_.midline.lifetime = rclcpp::Duration::max(); // 持续时间为最大，可以长时间显示在rviz中
        pnc_map_.midline.frame_locked = true;                // 地图是固定的

        // 左边界格式
        pnc_map_.left_boundary = pnc_map_.midline;
        pnc_map_.left_boundary.id = 1;
        pnc_map_.left_boundary.type = Marker::LINE_STRIP; // 连续线条（实线）
        pnc_map_.left_boundary.color.r = 1.0;
        pnc_map_.left_boundary.color.g = 1.0;
        pnc_map_.left_boundary.color.b = 1.0;

        // 右边界格式
        pnc_map_.right_boundary = pnc_map_.left_boundary;
        pnc_map_.right_boundary.id = 2;
    }

    void PNCMapCreatorStraight::draw_straight_x(const double &length, const double &plus_flag, const double &ratio)
    {
    }
} // namespace Planning
