#ifndef LOCALIZATION_KF_NAVIGATOR__TRAJECTORY_NAVIGATOR_HPP
#define LOCALIZATION_KF_NAVIGATOR__TRAJECTORY_NAVIGATOR_HPP

#include <chrono>
#include <memory>
#include <limits>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <limits>
#include <string>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <angles/angles.h>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"

#include "localization_kf_msgs/action/start_navigation.hpp"

#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include "px4_ros_com/frame_transforms.h"

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "localization_kf_navigator/trajectory_generator.hpp"


enum RobotState
{
  IDLE,
  PRE_NAVIGATING, // arm, takeoff to initial waypoint
  NAVIGATING,     // fly along the planned trajectory
  RETURNING_HOME,
  LANDING

};

namespace localization_kf_navigator
{

  /**
   * @class localization_kf_navigator::TrajectoryNavigator
   * @brief A class that based on a request from a service call, performs trajectory navigation.
   */
  class TrajectoryNavigator : public nav2_util::LifecycleNode
  {
  public:
    using StartNavigationAction = localization_kf_msgs::action::StartNavigation;
    using StartNavigationServer = nav2_util::SimpleActionServer<StartNavigationAction>;
    using NavSatFix = sensor_msgs::msg::NavSatFix;
    using VehicleCommand = px4_msgs::msg::VehicleCommand;
    using OffboardControlMode = px4_msgs::msg::OffboardControlMode;
    using TrajectorySetpoint = px4_msgs::msg::TrajectorySetpoint;
    using VehicleStatus = px4_msgs::msg::VehicleStatus;
    using VehicleOdometry = px4_msgs::msg::VehicleOdometry;
    using VehicleLocalPosition = px4_msgs::msg::VehicleLocalPosition;
    using Odometry = nav_msgs::msg::Odometry;
    using SensorGps = px4_msgs::msg::SensorGps;
    using Point = geometry_msgs::msg::Point;
    using VehicleAttitude = px4_msgs::msg::VehicleAttitude;
    using Path = nav_msgs::msg::Path;
    using Pose = geometry_msgs::msg::Pose;
    using TwistStamped = geometry_msgs::msg::TwistStamped;

    /**
     * @brief A constructor for the TrajectoryNavigator class.
     *
     */
    explicit TrajectoryNavigator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /**
     * @brief A destructor for the TrajectoryNavigator class.
     *
     */
    ~TrajectoryNavigator() = default;

  protected:
    /**
     * @brief Configure member variables and initializes planner
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;

    /**
     * @brief Activate member variables
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;

    /**
     * @brief Deactivate member variables
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;

    /**
     * @brief Reset member variables
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;

    /**
     * @brief Called when in shutdown state
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

    // action functions

    /**
     * @brief Handle start navigation action
     *
     */
    void handleStartNavigation();

    // callbacks
    /**
     * @brief Handle GPS data callback
     *
     * @param msg
     */
    void gpsCallback(const SensorGps::SharedPtr msg);

    /**
     * @brief Handle vehicle status callback
     *
     * @param msg
     */
    void vehicleStatusCallback(const VehicleStatus::SharedPtr msg);

    // /**
    //  * @brief Handle local position callback
    //  *
    //  * @param msg
    //  */
    // void localPositionCallback(const VehicleLocalPosition::SharedPtr msg);

    /**
     * @brief Handle vehicle odometry callback
     *
     * @param msg
     */
    void vehicleOdometryCallback(const VehicleOdometry::SharedPtr msg);

    // /**
    //  * @brief Handle vehicle attitude callback
    //  *
    //  * @param msg
    //  */
    // void vehicleAttitudeCallback(const VehicleAttitude::SharedPtr msg);

    // Other functions
    /**
     * @brief Publish vehicle command
     *
     * @param command
     * @param param1
     * @param param2
     * @param param3
     * @param param4
     * @param param5
     * @param param6
     * @param param7
     */
    void publishVehicleCommand(uint16_t command,
                               float param1 = 0.0,
                               float param2 = 0.0,
                               float param3 = 0.0,
                               float param4 = 0.0,
                               float param5 = 0.0,
                               float param6 = 0.0,
                               float param7 = 0.0);

    /**
     * @brief Publish trajectory setpoint
     *
     * @param x
     * @param y
     * @param z
     * @param yaw
     */
    void publishTrajectorySetpoint(float x, float y, float z, float yaw);

    /**
     * @brief Set offboard control mode
     *
     */
    void setOffboardControlMode();

    /**
     * @brief Set home position
     *
     */
    void setHomePosition();

    /**
     * @brief Wait for arm
     *
     */
    void waitForArm();

    /**
     * @brief Wait for disarm
     *
     */
    void waitForDisarm();

    /**
     * @brief Wait for takeoff
     *
     */
    void waitForTakeoff();

    /**
     * @brief Land
     *
     */
    void Land();

    /**
     * @brief Wait for offboard mode
     *
     */
    void waitForOffboardMode();

    /**
     * @brief Go to trajectory
     *
     * @param trajectory
     */
    void goToTrajectory(const Path &path);

    /**
     * @brief Check if the goal is reached
     *
     * @param set_point
     * @param yaw
     * @return true
     * @return false
     */
    bool reachedGoal(const Pose &set_pose);

    /**
     * @brief Get trajectory type string
     *
     * @param type
     * @return std::string
     */
    std::string get_trajectory_type_string(int type);

    /**
     * @brief Has timeout
     *
     * @return true
     * @return false
     */
    bool hasTimedOut(std::chrono::steady_clock::time_point start, double timeout)
    {
      return std::chrono::steady_clock::now() - start > std::chrono::seconds(static_cast<int>(timeout));
    }

    /**
     * @brief Switch back to hold mode
     *
     */
    void switchToHoldMode();

  private:
    // Actions
    std::shared_ptr<StartNavigationServer> _start_navigation_action_server;

    // publisher
    rclcpp_lifecycle::LifecyclePublisher<NavSatFix>::SharedPtr _gps_data_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<Odometry>::SharedPtr _odometry_publisher;
    rclcpp_lifecycle::LifecyclePublisher<VehicleCommand>::SharedPtr _vehicle_command_publisher;
    rclcpp_lifecycle::LifecyclePublisher<OffboardControlMode>::SharedPtr _offboard_control_mode_publisher;
    rclcpp_lifecycle::LifecyclePublisher<TrajectorySetpoint>::SharedPtr _trajectory_setpoint_publisher;
    rclcpp_lifecycle::LifecyclePublisher<Path>::SharedPtr _current_trajectory_publisher;
    rclcpp_lifecycle::LifecyclePublisher<TwistStamped>::SharedPtr _computed_cmd_vel_publisher;

    // subscribers
    rclcpp::Subscription<SensorGps>::SharedPtr _gps_subscription;
    rclcpp::Subscription<VehicleStatus>::SharedPtr _vehicle_status_subscription;
    // rclcpp::Subscription<VehicleLocalPosition>::SharedPtr _vehicle_local_position_subscription;
    // rclcpp::Subscription<VehicleAttitude>::SharedPtr _vehicle_attitude_subscription;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr _vehicle_odometry_subscription;

    // Transform broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

    // Variables
    NavSatFix _current_gps_data;
    NavSatFix _home_gps_position;
    Odometry _home_vehicle_odometry;
    VehicleStatus _current_vehicle_status;
    Odometry _current_vehicle_odometry;
    TrajectoryGenerator _trajectory_generator;
    VehicleAttitude _current_vehicle_attitude;
    RobotState _current_state;
    std::string _map_frame_id;
    std::string _gps_frame_id;
    std::string _base_frame_id;
    float _default_timeout{50.0};
    float _cruise_height{5.0};
    float _max_range{10.0};

    // Flags
    bool _gps_data_received;
    bool _odometry_data_received;
    bool _attitude_data_received;
    bool _home_position_set;
  };

} // namespace localization_kf_navigator

#endif // LOCALIZATION_KF_NAVIGATOR__TRAJECTORY_NAVIGATOR_HPP