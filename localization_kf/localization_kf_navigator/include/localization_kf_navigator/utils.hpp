#ifndef LOCALIZATION_KF_NAVIGATOR_UTILS_HPP
#define LOCALIZATION_KF_NAVIGATOR_UTILS_HPP

#include <math.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "rclcpp/rclcpp.hpp"

namespace localization_kf_navigator
{

    using Point = geometry_msgs::msg::Point;
    using Quaternion = geometry_msgs::msg::Quaternion;

    /**
     * @brief Return the geometry quaternion msg from two points
     *
     * @param point1
     * @param point2
     * @return float
     */

    Quaternion get_yaw_between_points_2d(const Point &point1, const Point &point2);

} // namespace localization_kf_navigator

#endif // LOCALIZATION_KF_NAVIGATOR_UTILS_HPP