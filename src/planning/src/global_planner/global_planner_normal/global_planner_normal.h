#ifndef GLOBAL_PLANNER_NORMAL_H_
#define GLOBAL_PLANNER_NORMAL_H_

#include "global_planner_base.h"

namespace Planning
{
    class GlobalPlannerNormal : public GlobalPlannerBase
    {
    public:
        GlobalPlannerNormal();

        Path search_global_path(const PNCMap &pnc_map) override; // 搜索全局路径
    };
} // namespace Planning
#endif // GLOBAL_PLANNER_NORMAL_H_
