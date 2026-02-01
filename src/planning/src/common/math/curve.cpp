#include "curve.h"

namespace Planning
{
    // 把角度约束到[-pi,pi)范围内。
    // 这里角度的统一很重要，如果两个角度数值上相差2pi,在做比较或运算时会判为不同的角度，但实际上是相同的角度。做一这里必须统一角度
    double Curve::NormalizeAngle(const double &angle)
    {
        double a = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (a < 0.0)
        {
            a += (2.0 * M_PI);
        }
        return a - M_PI;
    }

    // 笛卡尔转frenet
    void Curve::cartesian_to_frenet(
        // 输入1：目标点在笛卡尔下的参数：x, y, theta, kappa, speed, a
        const double &x, const double &y, const double &theta,
        const double &speed, const double &a, const double &kappa,
        // 输入2：目标点在参考线的投影点在笛卡尔下的参数：rs, rx, ry, rtheta, rkappa, rdkappa
        const double &rs, const double &rx, const double &ry,
        const double &rtheta, const double &rkappa, const double &rdkappa,
        // 输出：目标点在frenet下的参数：s, ds/dt, d(ds)/dt, l, dl/ds, dl/dt, d(dl)/ds, d(dl)/dt
        double &s, double &ds_dt, double &dds_dt,
        double &l, double &dl_ds, double &dl_dt, double &ddl_ds, double &ddl_dt)
    {
        // 计算s
        s = rs;

        // 计算l
        const double dx = x - rx;
        const double dy = y - ry;
        const double cos_theta_r = std::cos(rtheta);
        const double sin_theta_r = std::sin(rtheta);
        const double cross_r_x = cos_theta_r * dy - sin_theta_r * dx;
        l = std::copysign(std::hypot(dx, dy), cross_r_x);

        // 计算l' = dl/ds
        const double delta_theta = theta - rtheta;
        const double tan_delta_theta = std::tan(delta_theta);
        const double cos_delta_theta = std::cos(delta_theta);
        const double sin_delta_theta = std::sin(delta_theta);
        const double one_minus_kappa_l = 1 - rkappa * l;
        dl_ds = one_minus_kappa_l * tan_delta_theta;

        // 计算l'' = d(dl)/ds
        const double kappa_l_prime = rdkappa * l + rkappa * dl_ds;
        const double delta_theta_prime = one_minus_kappa_l / cos_delta_theta * kappa - rkappa;
        ddl_ds = -kappa_l_prime * tan_delta_theta +
                 one_minus_kappa_l / (cos_delta_theta * cos_delta_theta) * delta_theta_prime;

        // 计算ds/dt
        ds_dt = speed * cos_delta_theta / one_minus_kappa_l;

        // 计算d(ds)/dt
        dds_dt = (a * cos_delta_theta - (ds_dt * ds_dt) * (dl_ds * delta_theta_prime - kappa_l_prime)) / one_minus_kappa_l;

        // 计算dl_dt
        dl_dt = speed * sin_delta_theta;

        // 计算ddl_dt
        ddl_dt = a * sin_delta_theta;
    }

    // frenet转笛卡尔
    void Curve::frenet_to_cartesian( // 输出1：目标点在frenet下的参数：s, ds/dt, d(ds)/dt, l, dl/ds, d(dl)/ds
        const double &s, const double &ds_dt, const double &dds_dt,
        const double &l, const double &dl_ds, const double &ddl_ds,
        // 输入2：目标点在参考线的投影点在笛卡尔下的参数：rs, rx, ry, rtheta, rkappa, rdkappa
        const double &rs, const double &rx, const double &ry,
        const double &rtheta, const double &rkappa, const double &rdkappa,
        // 输出：目标点在笛卡尔下的参数：x, y, theta, kappa, speed, a
        double &x, double &y, double &theta,
        double &speed, double &a, double &kappa)
    {
    }

    // 找匹配点下标 (利用上一帧)
    int Curve::find_match_point(const Path &path, const int &last_match_point_index, const PoseStamped &target_point)
    {
        const int path_size = path.poses.size();
        if (path_size <= 1)
        {
            return path_size - 1;
        }

        double min_dis = std::numeric_limits<double>::max();
        int closest_index = -1;

        // 使用遍历找到最近点
        for (int i = 0; i < path_size; i++)
        {
            double dis = std::hypot(path.poses[i].pose.position.x - target_point.pose.position.x,
                                    path.poses[i].pose.position.y - target_point.pose.position.y);
            if (dis < min_dis)
            {
                if (abs(last_match_point_index - i) > 100)
                {
                    continue;
                }

                min_dis = dis;
                closest_index = i;
            }
        }

        return closest_index;
    }

    // 找匹配点下标（在参考线上）
    int Curve::find_match_point(const Referline &refer_line, const PoseStamped &target_point)
    {
        return 0;
    }

    // 找到投影点
    void Curve::find_projection_point(
        // 输入：参考线，目标点
        const Referline &path, const PoseStamped &target_point,
        // 输出：目标点在参考线的投影点在笛卡尔下的参数：rs, rx, ry, rtheta, rkappa, rdkappa
        double &rs, double &rx, double &ry,
        double &rtheta, double &rkappa, double &rdkappa)
    {
    }

    // 计算投影点参数（参考线）
    void Curve::cal_projection_param(Referline &refer_line)
    {
        const int path_size = refer_line.refer_line.size();
        if (path_size < 3)
        {
            RCLCPP_ERROR(rclcpp::get_logger("math"), "refer_line too short");
            return;
        }

        // 计算rs
        double rs = 0.0;
        for (int i = 0; i < path_size; i++)
        {
            if (i == 0)
            {
                rs = 0.0;
            }
            else
            {
                rs += std::hypot(refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i - 1].pose.pose.position.y,
                                 refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i - 1].pose.pose.position.x);
            }
            refer_line.refer_line[i].rs = rs;
        }

        // (计算航向角
        for (int i = 0; i < path_size; i++)
        {
            if (i < path_size - 1)
            {
                refer_line.refer_line[i].rtheta =
                    std::atan2(refer_line.refer_line[i + 1].pose.pose.position.y - refer_line.refer_line[i].pose.pose.position.y,
                               refer_line.refer_line[i + 1].pose.pose.position.x - refer_line.refer_line[i].pose.pose.position.x);
            }
            else
            { // 最后一个点
                refer_line.refer_line[i].rtheta =
                    std::atan2(refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i - 1].pose.pose.position.y,
                               refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i - 1].pose.pose.position.x);
            }
        }

        // 计算曲率
        for (int i = 0; i < path_size; i++)
        {
            if (i < path_size - 1)
            {
                const double dis = std::hypot(refer_line.refer_line[i + 1].pose.pose.position.y - refer_line.refer_line[i].pose.pose.position.y,
                                              refer_line.refer_line[i + 1].pose.pose.position.x - refer_line.refer_line[i].pose.pose.position.x);

                if (dis <= kMathEpsilon) // 如果算出来的结果非常接近0
                {
                    refer_line.refer_line[i].rkappa = 0.0;
                }
                else
                {
                    refer_line.refer_line[i].rkappa = (refer_line.refer_line[i + 1].rtheta - refer_line.refer_line[i].rtheta) / dis;
                }
            }
            else
            {
                const double dis = std::hypot(refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i - 1].pose.pose.position.y,
                                              refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i - 1].pose.pose.position.x);

                if (dis <= kMathEpsilon) // 如果算出来的结果非常接近0
                {
                    refer_line.refer_line[i].rkappa = 0.0;
                }
                else
                {
                    refer_line.refer_line[i].rkappa = (refer_line.refer_line[i].rtheta - refer_line.refer_line[i - 1].rtheta) / dis;
                }
            }
        }

        // 计算曲率变化率
        for (int i = 0; i < path_size; i++)
        {
            if (i < path_size - 1)
            {
                const double dis = std::hypot(refer_line.refer_line[i + 1].pose.pose.position.y - refer_line.refer_line[i].pose.pose.position.y,
                                              refer_line.refer_line[i + 1].pose.pose.position.x - refer_line.refer_line[i].pose.pose.position.x);

                if (dis <= kMathEpsilon) // 如果算出来的结果非常接近0
                {
                    refer_line.refer_line[i].rkappa = 0.0;
                }
                else
                {
                    refer_line.refer_line[i].rkappa = (refer_line.refer_line[i + 1].rkappa - refer_line.refer_line[i].rkappa) / dis;
                }
            }
            else
            {
                const double dis = std::hypot(refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i - 1].pose.pose.position.y,
                                              refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i - 1].pose.pose.position.x);

                if (dis <= kMathEpsilon) // 如果算出来的结果非常接近0
                {
                    refer_line.refer_line[i].rkappa = 0.0;
                }
                else
                {
                    refer_line.refer_line[i].rkappa = (refer_line.refer_line[i].rkappa - refer_line.refer_line[i - 1].rkappa) / dis;
                }
            }
        }
    }

} // namespace Planning
