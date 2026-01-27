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

        // 构造P矩阵
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
    }

} // namespace Planning