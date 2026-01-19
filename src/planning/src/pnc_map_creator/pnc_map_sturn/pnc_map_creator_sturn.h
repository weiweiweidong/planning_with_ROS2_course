#ifndef PNC_MAP_CREATOR_STURN_H_
#define PNC_MAP_CREATOR_STURN_H_

#include "pnc_map_creator_base.h"

namespace Planning
{
    class PNCMapCreatorSTurn : public PNCMapCreatorBase // 直道地图
    {
    public:
        PNCMapCreatorSTurn();
        PNCMap create_pnc_map() override; // 生成地图

    private:
    };
} // namespace Planning
#endif // PNC_MAP_CREATOR_STURN_H_
