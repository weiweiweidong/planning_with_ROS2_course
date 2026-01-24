#include "planning_process.h"

namespace Planning
{
  PlanningProcess::PlanningProcess() : Node("planning_process") // 规划总流程
  {
    RCLCPP_INFO(this->get_logger(), "planning_process created");

    // 读取配置文件
    process_config_ = std::make_unique<ConfigReader>();
    process_config_->read_planning_process_config();
    obs_dis_ = process_config_->process().obs_dis_;

    // 创建车辆和障碍物
    car_ = std::make_shared<MainCar>();

    // 创建广播器
    tf_broadcaster_ = std::make_shared<StaticTransformBroadcaster>(this);

    // 创建监听器
    buffer_ = std::make_unique<Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<TransformListener>(*buffer_, this);

    // 创建地图和全局路径客户端
    map_client_ = this->create_client<PNCMapService>("pnc_map_server");
    global_path_client_ = this->create_client<GlobalPathService>("global_path_server");

    // 创建 参考线 和 参考线的发布器
    refer_line_creator_ = std::make_shared<ReferenceLineCreator>();
    refer_line_pub_ = this->create_publisher<Path>("reference_line", 10);
  }

  bool PlanningProcess::process() // 总流程
  {
    // 阻塞1s，等待rviz2和xacro先启动
    rclcpp::Rate rate(1.0); // 频率为1Hz,即每一秒运行一次
    rate.sleep();

    // 初始化
    if (!planning_init())
    {
      // 如果初始化失败，就报错
      RCLCPP_ERROR(this->get_logger(), "planning init failed");
      return false;
    }

    // 进入规划主流程
    timer_ = this->create_wall_timer( // 规划模块会以0.1s为周期去运行
        0.1s,                         // 0.1s为周期
        std::bind(&PlanningProcess::planning_callback, this));

    return true;
  }
  bool PlanningProcess::planning_init() // 流程初始化
  {
    // 生成车辆
    vehicle_spawn(car_);

    // 连接地图服务器
    if (!connect_server(map_client_))
    {
      RCLCPP_ERROR(this->get_logger(), "Map server connect failed!");
      return false;
    }

    // 获取地图
    if (!map_request())
    {
      RCLCPP_ERROR(this->get_logger(), "Map request and response failed!");
      return false;
    }

    // 连接全局规划服务器
    if (!connect_server(global_path_client_))
    {
      RCLCPP_ERROR(this->get_logger(), "Global_path server connect failed!");
      return false;
    }

    // 获取全局路径
    if (!global_path_request())
    {
      RCLCPP_ERROR(this->get_logger(), "Global_path request and response failed!");
      return false;
    }

    return true;
  }

  // 生成车辆
  void PlanningProcess::vehicle_spawn(const std::shared_ptr<VehicleBase> &vehicle)
  {
    TransformStamped spawn;
    spawn.header.stamp = this->now();
    spawn.header.frame_id = process_config_->pnc_map().frame_; // 地图坐标
    spawn.child_frame_id = vehicle->child_frame();             // 地图坐标的子坐标设为车辆坐标

    spawn.transform.translation.x = vehicle->loc_point().pose.position.x;
    spawn.transform.translation.y = vehicle->loc_point().pose.position.y;
    spawn.transform.translation.z = vehicle->loc_point().pose.position.z;
    spawn.transform.rotation.x = vehicle->loc_point().pose.orientation.x;
    spawn.transform.rotation.y = vehicle->loc_point().pose.orientation.y;
    spawn.transform.rotation.z = vehicle->loc_point().pose.orientation.z;
    spawn.transform.rotation.w = vehicle->loc_point().pose.orientation.w;

    RCLCPP_INFO(this->get_logger(), "vehicle %s spawned, x = %.2f, y = %.2f",
                spawn.child_frame_id.c_str(),
                vehicle->loc_point().pose.position.x,
                vehicle->loc_point().pose.position.y);
    // 把赋完值的参数广播出去
    tf_broadcaster_->sendTransform(spawn);
  }

  // 监听定位点
  void PlanningProcess::get_location(const std::shared_ptr<VehicleBase> &vehicle)
  {
    try
    {
      PoseStamped point;
      // 监听到当前时间下面的父坐标和子坐标
      auto ts = buffer_->lookupTransform(process_config_->pnc_map().frame_, vehicle->child_frame(), tf2::TimePointZero);

      // 赋值
      point.header = ts.header;
      point.pose.position.x = ts.transform.translation.x;
      point.pose.position.y = ts.transform.translation.y;
      point.pose.position.z = ts.transform.translation.z;
      point.pose.orientation.x = ts.transform.rotation.x;
      point.pose.orientation.y = ts.transform.rotation.y;
      point.pose.orientation.z = ts.transform.rotation.z;
      point.pose.orientation.w = ts.transform.rotation.w;
      // 更新数据
      vehicle->update_location(point);
    }
    catch (const tf2::LookupException &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Lookup Exception: %s", e.what());
    }
  }

  template <typename T>
  bool PlanningProcess::connect_server(const T &client) // 连接服务器
  {
    // 判断客户端类型
    std::string server_name;
    if constexpr (std::is_same_v<T, rclcpp::Client<PNCMapService>::SharedPtr>) // C++ 17 引入的特性
    {
      server_name = "pnc_map";
    }
    else if constexpr (std::is_same_v<T, rclcpp::Client<GlobalPathService>::SharedPtr>)
    {
      server_name = "global_path";
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "wrong client type!");
      return false;
    }

    // 等待服务器
    while (!client->wait_for_service(1s))
    {
      if (!rclcpp::ok()) // 对ctrl+c操作处理，防止进入死循环
      {
        RCLCPP_ERROR(this->get_logger(), "Interruped while waiting for the %s server.", server_name.c_str());
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "%s server not available, waiting again...", server_name.c_str());
    }

    return true;
  }

  bool PlanningProcess::map_request() // 发送地图请求
  {
    RCLCPP_INFO(this->get_logger(), "Sending map request");

    // 生成请求
    auto request = std::make_shared<PNCMapService::Request>();
    request->map_type = process_config_->pnc_map().type_;

    // 获取响应
    auto result_future = map_client_->async_send_request(request);

    // 判断响应是否成功
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Map response success");
      pnc_map_ = result_future.get()->pnc_map; // 获取响应中的pnc_map
      return true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Map response failed!");
      return false;
    }

    return true;
  }

  bool PlanningProcess::global_path_request() // 发送全局路径请求
  {
    RCLCPP_INFO(this->get_logger(), "Sending global_path request");

    // 生成请求
    auto request = std::make_shared<GlobalPathService::Request>();
    request->pnc_map = pnc_map_;
    request->global_planner_type = process_config_->global_path().type_;

    // 获取响应
    auto result_future = global_path_client_->async_send_request(request);

    // 判断响应是否成功
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "global_path response success");
      global_path_ = result_future.get()->global_path; // 获取响应中的pnc_map
      return true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Map response failed!");
      return false;
    }

    return true;
  }

  // 总流程回调
  void PlanningProcess::planning_callback()
  {
  }
}