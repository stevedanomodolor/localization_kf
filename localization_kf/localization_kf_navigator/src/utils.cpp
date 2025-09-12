#include "localization_kf_navigator/utils.hpp"

namespace localization_kf_navigator
{
    Quaternion get_yaw_between_points_2d(const Point &point1, const Point &point2)
    {
        float delta_x = point2.x - point1.x;
        float delta_y = point2.y - point1.y;
        float yaw = atan2(delta_y, delta_x);
        Quaternion quat;
        quat.z = sin(yaw / 2);
        quat.w = cos(yaw / 2);
        return quat;
    }

} // namespace localization_kf_navigator
