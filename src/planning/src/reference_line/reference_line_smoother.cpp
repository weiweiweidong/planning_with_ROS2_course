#include "reference_line_smoother.h"

namespace Planning
{
    ReferenceLineSmoother::ReferenceLineSmoother() // 参考线平滑
    {
        RCLCPP_INFO(rclcpp::get_logger("reference_line"), "reference_line_smoother created");

        // 读取配置文件
        reference_line_config_ = std::make_unique<ConfigReader>();
        reference_line_config_->read_reference_line_config();
    }

    void ReferenceLineSmoother::smooth_reference_line(Referline &refer_line) // 平滑参考线
    {
        const int n = refer_line.refer_line.size();
        if (n < 3) // 如果点太少，我们认为这条参考线为空/过短 （没有平滑的必要）
        {
            return;
        }

        // ======================================= Part 1：构造要用到的各个矩阵 =======================================
        // ---------------------------- 1.1 构造组装P矩阵用的各个组件 ----------------------------
        Eigen::Matrix2d I = Eigen::Matrix2d::Identity(); // 2*2单位阵
        Eigen::Matrix2d W1 = 2.0 * w1 * I;
        Eigen::Matrix2d W2 = 2.0 * w2 * I;
        Eigen::Matrix2d W3 = 2.0 * w3 * I;

        Eigen::Matrix2d block1 = W1 + W2 + W3;
        Eigen::Matrix2d block2 = -2.0 * W1 - W2;
        Eigen::Matrix2d block3 = -4.0 * W1 - W2;
        Eigen::Matrix2d block4 = 5.0 * W1 + 2.0 * W2 + W3;
        Eigen::Matrix2d block5 = 6.0 * W1 + 2.0 * W2 + W3;

        Eigen::MatrixXd P_tmp = Eigen::MatrixXd::Zero(2 * n, 2 * n);

        // ---------------------------- 1.2 组装P的上三角阵 ----------------------------
        if (n == 3) // 单独处理n==3的情况
        {
            // 上三角部分：
            // |W1+W2+W3  -2W1-W2     W1       |
            // |          4W1+2W2+W3  -2W1-W2  |
            // |                      W1+W2+W3 |

            // 只填充上三角部分
            P_tmp.block<2, 2>(0, 0) = block1; // 从左上角0行0列的位置填充
            P_tmp.block<2, 2>(0, 2) = block2; // 从左上角0行2列的位置填充
            P_tmp.block<2, 2>(0, 4) = W1;
            P_tmp.block<2, 2>(2, 2) = 4.0 * W1 + 2.0 * W2 + W3;
            P_tmp.block<2, 2>(2, 4) = block2;
            P_tmp.block<2, 2>(4, 4) = block1;
        }
        else
        {
            // 上三角部分：
            // |W1+W2+W3  -2W1-W2     W1          0            0            0       |
            // |          5W1+2W2+W3  -4W1-W2     W1           0            0       |
            // |                      6W1+2W2+W3  -4W1-W2      W1           0       |
            // |                                  .           .             .       |
            // |                                  .           .             .       |
            // |                                  .           .             .       |
            // |                                  6W1+2W2+W3  -4W1-W2     W1        |
            // |                                              5W1+2W2+W3  -2W1-W2   |
            // |                                                          W1+W2+W3  |

            // 只填充上三角部分
            for (int i = 0; i < n; i++) // P矩阵维度为2n*2n。但是是按照2*2的小矩阵来填的，所以这里P矩阵被视作一个n*n的大矩阵
            {
                if (i == 0) // 第0行
                {
                    P_tmp.block<2, 2>(i * 2, i * 2) = block1;
                    P_tmp.block<2, 2>(i * 2, (i + 1) * 2) = block2;
                    P_tmp.block<2, 2>(i * 2, (i + 2) * 2) = W1;
                }
                else if (i == 1) // 第1行
                {
                    P_tmp.block<2, 2>(i * 2, i * 2) = block4;
                    P_tmp.block<2, 2>(i * 2, (i + 1) * 2) = block3;
                    P_tmp.block<2, 2>(i * 2, (i + 2) * 2) = W1;
                }
                else if (i == n - 2) // 倒数第2航
                {
                    P_tmp.block<2, 2>(i * 2, i * 2) = block4;
                    P_tmp.block<2, 2>(i * 2, (i + 1) * 2) = block2;
                }
                else if (i == n - 1) // 最后1行
                {
                    P_tmp.block<2, 2>(i * 2, i * 2) = block1;
                }
                else // 中间行
                {
                    P_tmp.block<2, 2>(i * 2, i * 2) = block5;
                    P_tmp.block<2, 2>(i * 2, (i + 1) * 2) = block3;
                    P_tmp.block<2, 2>(i * 2, (i + 2) * 2) = W1;
                }
            }
        }

        // ---------------------------- 1.3 构建P矩阵 ----------------------------
        P_tmp = P_tmp.selfadjointView<Eigen::Upper>();      // 通过上三角阵构造对称阵
        Eigen::SparseMatrix<double> P = P_tmp.sparseView(); // 转化为稀疏矩阵

        // ---------------------------- 1.4 构建A矩阵 ----------------------------
        Eigen::MatrixXd A_tmp = Eigen::MatrixXd::Identity(2 * n, 2 * n);
        Eigen::SparseMatrix<double> A = A_tmp.sparseView(); // 转化为稀疏矩阵

        // ---------------------------- 1.5 构建X和Q ----------------------------
        Eigen::VectorXd X(2 * n);
        for (int i = 0; i < n; i++)
        {
            X(i * 2) = refer_line.refer_line[i].pose.pose.position.x;
            X(i * 2 + 1) = refer_line.refer_line[i].pose.pose.position.y;
        }

        Eigen::VectorXd Q = -2.0 * X; // 一次项矩阵

        // ---------------------------- 1.6 构建偏差量buff ----------------------------
        Eigen::VectorXd buff = Eigen::VectorXd::Constant(2 * n, 0.2); // 偏差范围，动态列向量，2*n行，值全为0.2
        buff(0) = buff(1) = buff(2 * n - 2) = buff(2 * n - 1) = 0;    // 首尾点坐标不变

        // ---------------------------- 1.7 构建上下边界UB和LB ----------------------------
        Eigen::VectorXd lowerBound = X - buff; // 不等式约束的下边界
        Eigen::VectorXd upperBound = X + buff; // 不等式约束的上边界

        // ======================================= Part 2：求解 =======================================
        // ---------------------------- 2.1 创建求解器 ----------------------------
        // 创建求解器
        OsqpEigen::Solver solver;

        // 设置
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);

        // ---------------------------- 2.2 求解器初始化 ----------------------------
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

        // ---------------------------- 2.3 求解 ----------------------------
        Eigen::VectorXd QPSolution;                                     // 二次规划的结果 （待求解的值）
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) // 如果有报错，就return
        {
            return;
        }

        QPSolution = solver.getSolution(); // 获取解

        // 把结果向量中的数据更新到refer_line中
        for (int i = 0; i < n; i++)
        {
            refer_line.refer_line[i].pose.pose.position.x = QPSolution(i * 2);
            refer_line.refer_line[i].pose.pose.position.y = QPSolution(i * 2 + 1);
        }
    }

} // namespace Planning