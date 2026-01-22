#include "curve.h"

namespace Planning
{

    // 找匹配点下标
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

} // namespace Planning
