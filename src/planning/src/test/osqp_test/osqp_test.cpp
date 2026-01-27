#include "osqp_test.h"

namespace Planning
{
    OsqpTest::OsqpTest() : Node("osqp_test_node") // osqp测试
    {
        RCLCPP_INFO(this->get_logger(), "osqp_test_node created");

        test_problem();
    }
    void OsqpTest::test_problem() // 示例问题
    {
    }
} // namespace Planning

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planning::OsqpTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}