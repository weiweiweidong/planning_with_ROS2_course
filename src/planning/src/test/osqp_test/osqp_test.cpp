#include "osqp_test.h"

namespace Planning
{
    OsqpTest::OsqpTest() : Node("osqp_test_node") // osqp测试
    {
        RCLCPP_INFO(this->get_logger(), "osqp_test_node created");

        test_problem();
    }
    void OsqpTest::test_problem() // QP求解的示例问题
    {
        Eigen::SparseMatrix<double> P(2, 2); // P, 二次型矩阵 （声明一个尺寸为2*2的方阵）
        Eigen::VectorXd Q(2);                // Q, 一次项矩阵 （声明一个动态向量，尺寸为2。（即声明一个2行的列向量））
        Eigen::SparseMatrix<double> A(2, 2); // 单位阵
        Eigen::VectorXd lowerBound(2);       // 下边界向量
        Eigen::VectorXd upperBound(2);       // 上边界向量

        // -------------------------- 1.矩阵变量声明 --------------------------
        P.insert(0, 0) = 2.0; // 0行0列位置，值为2.0
        P.insert(1, 1) = 2.0; // 1行1列位置，值为2.0
        std::cout << "P:" << std::endl
                  << P << std::endl;

        Q << -2, -2;
        std::cout << "Q:" << std::endl
                  << Q << std::endl;

        A.insert(0, 0) = 1.0;
        A.insert(1, 1) = 1.0;
        std::cout << "A:" << std::endl
                  << A << std::endl;

        lowerBound << 0.0, 0.0;
        upperBound << 1.5, 1.5;

        // -------------------------- 2.创建求解器 --------------------------

        // 创建求解器
        OsqpEigen::Solver solver;

        // 设置
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);

        // -------------------------- 3.求解器初始化 --------------------------
        solver.data()->setNumberOfVariables(2);   // 变量数
        solver.data()->setNumberOfConstraints(2); // 约束数 （这里理解为LB和UB的行数，为2）
        if (!solver.data()->setHessianMatrix(P))  // 设置海塞矩阵（有可能会失败。如果失败就return掉）
        {
            return;
        }
        if (!solver.data()->setGradient(Q))
        {
            return;
        }
        if (!solver.data()->setLinearConstraintsMatrix(A)) // 设置线性约束
        {
            return;
        }
        if (!solver.data()->setLowerBound(lowerBound)) // 设置下边界
        {
            return;
        }
        if (!solver.data()->setUpperBound(upperBound)) // 设置上边界
        {
            return;
        }
        if (!solver.initSolver())
        {
            return;
        }

        // -------------------------- 4.求解 --------------------------
        Eigen::VectorXd QPSolution;                                     // 二次规划的结果 （待求解的值）
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) // 如果有报错，就return
        {
            return;
        }

        QPSolution = solver.getSolution(); // 获取解
        std::cout << "QPSolution:" << std::endl
                  << QPSolution << std::endl;
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