#include "decision_center.h"

namespace Planning
{
    DecisionCenter::DecisionCenter()
    {
        RCLCPP_INFO(rclcpp::get_logger("decision_center"), "decision_center created");

        // 读取配置文件
        decision_config_ = std::make_unique<ConfigReader>();
        decision_config_->read_decision_config(); // 读一下决策模块的配置
    }

    // 路径决策
    void DecisionCenter::make_path_decision(const std::shared_ptr<VehicleBase> &car,
                                            const std::vector<std::shared_ptr<VehicleBase>> &obses)
    {
        if (obses.empty()) // 没有障碍物，就不需要决策。直接return
        {
            return;
        }

        // 初始化
        sl_points_.clear();
        const double left_bound_l = decision_config_->pnc_map().road_half_width_ * 1.5;              // 道路左边界
        const double right_bound_l = -decision_config_->pnc_map().road_half_width_ / 2.0;            // 道路右边界
        const double dis_time = static_cast<double>(decision_config_->local_path().path_size_ - 50); // 开始考虑障碍物的范围  (80-50,相当于提前30个点开始考虑变道)
        const double least_length = std::max(car->ds_dt() * dis_time, 30.0);                         // 最小变道距离 （最小变道距离和车速挂钩。但是同时又保证车速很慢的时候，不至于靠得很近才开始变道）
        const double referline_end_length = decision_config_->refer_line().front_size_ *
                                            decision_config_->pnc_map().segment_len_; // 参考线前段的长度的最大值
        SLPoint p;                                                                    // 虚拟障碍物的点位

        // 针对每个障碍物计算变道点位
        for (const auto &obs : obses)
        {
            const double obs_dis_s = obs->s() - car->s(); // 与障碍物的距离
            if (obs_dis_s > referline_end_length ||       // 如果障碍物在参考线末端的前面
                obs_dis_s < -least_length                 //
            )
            {
                // 上面两个判断条件是什么意思？
                // 判断条件1：即使接近地图终点，参考线变短，也要考虑最长距离，防止碰撞
                // 判断条件2：车辆越过障碍物变回主车道，和后方的障碍物要保持安全距离
                continue; // 跳过。即 不管当前障碍物
            }

            if (obs->l() > right_bound_l && obs->l() < left_bound_l &&              // 如果障碍物在车道横向中间 (会车场景)
                fabs(obs->dl_dt()) < min_speed && obs->ds_dt() < car->ds_dt() / 2.0 // 侧向速度为0,纵向速度慢（超车场景，障碍物车速比主车慢）
            )
            {
                // 计算虚拟障碍物的位置
                p.s_ = obs->s() + obs->ds_dt() * obs_dis_s / (car->ds_dt() - obs->ds_dt());
                // 计算虚拟障碍物的左右边界
                const double obs_left_bound_l = obs->l() + obs->width() / 2.0;  // 障碍物左边界
                const double obs_right_bound_l = obs->l() - obs->width() / 2.0; // 障碍物右边界
                const double left_width = left_bound_l - obs_left_bound_l;      // 左边宽度
                const double right_width = obs_right_bound_l - right_bound_l;   // 右边宽度

                // 如果左边宽度能够通过
                if (left_width > car->width() + decision_config_->decision().safe_dis_l_ * 2.0) // 左边宽度要 大于 主车宽度+2边的安全距离
                {
                    p.l_ = (left_bound_l + obs_left_bound_l) / 2.0;
                    p.type_ = static_cast<int>(SLPointType::LEFT_PASS); // 类型设为“左边绕”
                    sl_points_.emplace_back(p);
                }
                else // 如果左边宽度不够
                {
                    // 如果右边宽度能够通过
                    if (right_width > car->width() + decision_config_->decision().safe_dis_l_ * 2.0)
                    {
                        p.l_ = (right_bound_l + obs_right_bound_l) / 2.0;
                        p.type_ = static_cast<int>(SLPointType::RIGHT_PASS); // 类型设为“右边绕”
                        sl_points_.emplace_back(p);
                    }
                    else // 如果两边宽度都不够：绕不过去要停障
                    {
                        p.l_ = 0.0;
                        p.s_ = obs->s() - decision_config_->decision().safe_dis_s_;
                        p.type_ = static_cast<int>(SLPointType::STOP); // 类型设为“停止”
                        sl_points_.emplace_back(p);
                        RCLCPP_INFO(rclcpp::get_logger("decision_center"), "--------------stop obs, p:(s=%.2f, l=%.2f)", p.s_, p.l_);
                        break; // 既然停止了，就不需要考虑更前方的障碍物了
                    }
                }
            }
        }

        // 双保险：如果没有障碍物，就沿着参考线继续走就行，不需要进行路径决策。相当于是把本函数最开始的if判断又写了一遍
        if (sl_points_.empty())
        {
            return;
        }

        // 头尾的处理
        SLPoint p_start;                              // 整个过程的起点
        p_start.s_ = sl_points_[0].s_ - least_length; // 容器被排序过，所以index[0]的就是最近的障碍物
        p_start.l_ = 0.0;
        p_start.type_ = static_cast<int>(SLPointType::START);
        sl_points_.emplace(sl_points_.begin(), p_start); // 头插（把起始点插到序列的最前面）

        if (sl_points_.back().type_ != static_cast<int>(SLPointType::STOP))
        {
            SLPoint p_end;
            p_end.s_ = sl_points_.back().s_ + least_length;
            p_end.l_ = 0.0;
            p_end.type_ = static_cast<int>(SLPointType::END);
            sl_points_.emplace_back(p_end); // 尾插
        }
    }

    // 速度决策
    void DecisionCenter::make_speed_decision(const std::shared_ptr<VehicleBase> &car,
                                             const std::vector<std::shared_ptr<VehicleBase>> &obses)
    {
        if (obses.empty()) // 没有障碍物
        {
            return;
        }

        st_points_.clear();
        const double ori_dis_time = static_cast<double>(decision_config_->local_speeds().speed_size_ - 50);                      // 开始考虑障碍物的时间，50帧
        const double ori_dis = ori_dis_time * decision_config_->main_car().speed_ori_;                                           // 开始考虑障碍物的距离，50米
        const double real_brake_time = (ori_dis_time + static_cast<double>(decision_config_->local_speeds().speed_size_)) / 2.0; // 真实的制动时间。（取50和100的中间值，即75）
        STPoint p;

        // 针对每个障碍物计算变速点位
        for (const auto &obs : obses)
        {
            const double obs_dis_s = obs->s_2path() + car->speed();    // 主车与障碍物的距离（沿路径）
            if (obs_dis_s > ori_dis ||                                 // 障碍物车辆距离太远，现在还不用考虑
                obs_dis_s < -decision_config_->decision().safe_dis_s_) // 障碍物已经落后主车太远，超过了安全距离
            {
                continue;
            }

            double t_in;  // 切入时间
            double t_out; // 切出时间

            if (fabs(obs->l_2path()) < obs->width() / 2.0) // 如果障碍物已经占据了路径。（障碍物中心到路径的横向距离，小于障碍物宽度的一半）
            {
                if (fabs(obs->dl_dt_2path()) < min_speed) // 并且侧向速度很低
                {
                    if (obs->ds_dt_2path() > car->speed() + 0.5) // 如果障碍物纵向车速大于主车
                    {
                        continue; // 忽略
                    }

                    // ============================== 跟车 ==============================
                    // 计算初始st,切入和切出的时间
                    obs->update_t0(); // 计时
                    p.t0_ = obs->t0();
                    p.s0_ = obs_dis_s + obs->ds_dt_2path() * p.t0_ - ori_dis;
                    t_in = 0.0;
                    t_out = decision_config_->local_speeds().speed_size_;
                    RCLCPP_INFO(rclcpp::get_logger("decision_center"), "------------obs_dis_s = %.2f, p.t0 = %.2f, p.s0 = %.2f, t_in = %.2f, t_out = %.2f",
                                obs_dis_s, p.t0_, p.s0_, t_in, t_out);

                    // 计算st点
                    p.t_ = p.t0_ + real_brake_time;
                    p.s_2path_ = obs_dis_s - decision_config_->decision().safe_dis_s_ + obs->ds_dt_2path() * p.t_;
                    p.ds_dt_2path_ = obs->ds_dt_2path();
                    p.type_ = static_cast<int>(STPointType::STOP);
                    st_points_.emplace_back(p);
                    RCLCPP_INFO(rclcpp::get_logger("decision_center"), "------------stop or follow obs: p:(s=%.2f, t=%.2f, ds_dt=%.2f)",
                                p.s_2path_, p.t_, p.ds_dt_2path_);

                    obs->update_t_in_out(p.t_, t_in, t_out);
                    break;
                }
            }
            else // 障碍物没有占据路径
            {
                if (fabs(obs->dl_dt_2path()) < min_speed) // 如果没有侧向速度
                {
                    continue;
                }

                if (decision_config_->main_car().speed_ori_ < min_speed) // 如果主车速度很低，则无需让速或抢行
                {
                    continue;
                }

                const double car_dis_time = obs_dis_s / decision_config_->main_car().speed_ori_; // 主车中心到达障碍物S位置的时间
                const double obs_dis_time = (0.0 - obs->l_2path()) / obs->dl_dt_2path();         // 障碍物中心到达路径的时间
                if (obs_dis_time < 0.0)                                                          // 说明是远离路径的方向
                {
                    continue;
                }

                // 确定初始s和t
                obs->update_t0(); // 计时
                p.t0_ = obs->t0();
                p.s0_ = obs_dis_s - ori_dis;

                // 计算切入和切出时间
                const double delta_t = decision_config_->decision().safe_dis_s_ / decision_config_->main_car().speed_ori_; // 安全时间
                const double half_through_time = fabs(obs->length() / 2.0 / obs->dl_dt_2path());                           // 半个车身穿过路径的时间
                t_in = obs_dis_time - half_through_time;
                t_out = obs_dis_time + half_through_time;

                RCLCPP_INFO(rclcpp::get_logger("decision_center"), "------------obs_dis_s = %.2f, p.t0 = %.2f, p.s0 = %.2f, car_dis_time = %.2f , obs_dis_time = %.2f , t_in = %.2f , t_out = %.2f, delta_t = %.2f",
                            obs_dis_s, p.t0_, p.s0_, car_dis_time, obs_dis_time, t_in, t_out, delta_t);

                // 计算st点
                if (car_dis_time > obs_dis_time && car_dis_time < t_out + delta_t) // 让行
                {
                    p.t_ = t_out;
                    p.s_2path_ = obs_dis_s - decision_config_->decision().safe_dis_s_;
                    p.ds_dt_2path_ = decision_config_->main_car().speed_ori_;
                    p.type_ = static_cast<int>(STPointType::GIVE_WAY);
                    st_points_.emplace_back(p);
                    RCLCPP_INFO(rclcpp::get_logger("decision_center"), "------------give way, p:(s=%.2f, t=%.2f, ds_dt=%.2f)",
                                p.s_2path_, p.t_, p.ds_dt_2path_);
                }
                else if (car_dis_time < obs_dis_time && car_dis_time > t_in - delta_t) // 抢行
                {
                    p.t_ = t_in;
                    p.s_2path_ = obs_dis_s + decision_config_->decision().safe_dis_s_;
                    p.ds_dt_2path_ = decision_config_->main_car().speed_ori_;
                    p.type_ = static_cast<int>(STPointType::RUSH_OUT);
                    st_points_.emplace_back(p);
                    RCLCPP_INFO(rclcpp::get_logger("decision_center"), "------------rush out, p:(s=%.2f, t=%.2f, ds_dt=%.2f)",
                                p.s_2path_, p.t_, p.ds_dt_2path_);
                }

                obs->update_t_in_out(p.t_, t_in, t_out);
            }
        }

        if (st_points_.empty())
        {
            return;
        }

        // 整个过程的起点
        STPoint p_start;
        p_start.t_ = st_points_[0].t0_;
        p_start.s_2path_ = st_points_[0].s0_;
        p_start.ds_dt_2path_ = decision_config_->main_car().speed_ori_;
        p_start.type_ = static_cast<int>(STPointType::START);
        st_points_.emplace(st_points_.begin(), p_start); // 头插

        // 整个过程的终点
        STPoint p_end;
        p_end.t_ = decision_config_->local_speeds().speed_size_;
        p_end.s_2path_ = st_points_.back().s_2path_ + st_points_.back().ds_dt_2path_ * (p_end.t_ - st_points_.back().t_);
        p_end.ds_dt_2path_ = st_points_.back().ds_dt_2path_;
        p_end.type_ = static_cast<int>(STPointType::END);
        st_points_.emplace_back(p_end); // 头插

        RCLCPP_INFO(rclcpp::get_logger("decision_center"), "------------p_start: (s=%.2f, t=%.2f, ds_dt=%.2f), p_end: (s=%.2f, t=%.2f, ds_dt=%.2f)",
                    p_start.s_2path_, p_start.t_, p_start.ds_dt_2path_,
                    p_end.s_2path_, p_end.t_, p_end.ds_dt_2path_);
    }
}