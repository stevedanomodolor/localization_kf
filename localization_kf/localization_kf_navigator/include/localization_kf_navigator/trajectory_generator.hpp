#ifndef LOCALIZATION_KF_NAVIGATOR_TRAJECTORY_GENERATOR_HPP
#define LOCALIZATION_KF_NAVIGATOR_TRAJECTORY_GENERATOR_HPP

#include <vector>
#include <math.h>
#include <string>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "localization_kf_navigator/utils.hpp"

enum class TrajectoryType
{
  CIRCLE,
  SQUARE,
  FIGURE8
};

namespace localization_kf_navigator
{

  /**
   * @class localization_kf_navigator::TrajectoryGenerator
   * @brief A class that generates trajectories based on the type and current position.
   */

  class TrajectoryGenerator
  {
  public:
    using Path = nav_msgs::msg::Path;
    using Pose = geometry_msgs::msg::Pose;
    using PoseStamped = geometry_msgs::msg::PoseStamped;

    /**
     * @brief Construct a new Trajectory Generator object
     *
     */
    TrajectoryGenerator(std::string map_frame_id = "map", float max_range = 1000);

    /**
     * @brief Generate a trajectory based on the type and reference position.
     *
     * @param type
     * @param ref_position
     * @return Path
     */
    Path generateTrajectory(const int &type, const Pose &ref_pose);

  private:
    /**
     * @brief Generate a square trajectory.
     *
     * @param ref_pose
     * @return Path
     */
    Path generateSquareTrajectory(const Pose &ref_pose);

    /**
     * @brief Generate a circular trajectory.
     *
     * @param ref_pose
     * @return Path
     */
    Path generateCircleTrajectory(const Pose &ref_pose);

    /**
     * @brief Generate a figure-8 trajectory.
     *
     * @param ref_pose
     * @return Path
     */
    Path generateFigure8Trajectory(const Pose &ref_pose);

    float _max_range;
    std::string _map_frame_id;
  };

} // namespace localization_kf_navigator

#endif // LOCALIZATION_KF_NAVIGATOR_TRAJECTORY_GENERATOR_HPP