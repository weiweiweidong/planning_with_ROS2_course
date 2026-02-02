#include "polynomial_curve.h"

namespace Planning
{

    // 五次多项式
    Eigen::Vector<double, 6> PolynomialCurve::quintic_polynomial(const double &start_x, const double &start_y,
                                                                 const double &start_dy_dx, const double &start_ddy_dx,
                                                                 const double &end_x, const double &end_y,
                                                                 const double &end_dy_dx, const double &end_ddy_dx)
    {
        // 预先计算出来2次方，3次方，4次方，5次方的值备用
        const double start_x_2 = start_x * start_x;
        const double start_x_3 = start_x_2 * start_x;
        const double start_x_4 = start_x_3 * start_x;
        const double start_x_5 = start_x_4 * start_x;
        const double end_x_2 = end_x * end_x;
        const double end_x_3 = end_x_2 * end_x;
        const double end_x_4 = end_x_3 * end_x;
        const double end_x_5 = end_x_4 * end_x;

        // 套计算公式创建矩阵S
        Eigen::Matrix<double, 6, 6> S;
        S << 1.0, start_x, start_x_2, start_x_3, start_x_4, start_x_5,
            0.0, 1.0, 2.0 * start_x, 3.0 * start_x_2, 4.0 * start_x_3, 5.0 * start_x_4,
            0.0, 0.0, 2.0, 6.0 * start_x, 12.0 * start_x_2, 20.0 * start_x_3,
            1.0, end_x, end_x_2, end_x_3, end_x_4, end_x_5,
            0.0, 1.0, 2.0 * end_x, 3.0 * end_x_2, 4.0 * end_x_3, 5.0 * end_x_4,
            0.0, 0.0, 2.0, 6.0 * end_x, 12.0 * end_x_2, 20.0 * end_x_3;

        // 套公式创建向量L
        Eigen::Vector<double, 6> L;
        L << start_y, start_dy_dx, start_ddy_dx, end_y, end_dy_dx, end_ddy_dx;

        // 求解五次多项式的系数向量
        return S.colPivHouseholderQr().solve(L);
    }

} // namespace Planning
