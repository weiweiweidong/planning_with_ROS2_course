#ifndef OBS_CAR_BASE_H_
#define OBS_CAR_BASE_H_

#include "vehicle_info_base.h"

namespace Planning
{
    class ObsCar : public VehicleBase // 障碍车
    {
    public:
        ObsCar(const int &id);
    };
} // namespace Planning
#endif // OBS_CAR_BASE_H_
