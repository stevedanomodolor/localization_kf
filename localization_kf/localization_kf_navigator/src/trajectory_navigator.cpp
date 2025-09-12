#include "localization_kf_navigator/trajectory_navigator.hpp"

namespace localization_kf_navigator
{

  TrajectoryNavigator::TrajectoryNavigator(const rclcpp::NodeOptions &options)
      : nav2_util::LifecycleNode("trajectory_navigator", "", options)
  {
  }

  nav2_util::CallbackReturn TrajectoryNavigator::on_configure(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Configuring localization_kf_navigator...");

    // set flags
    _odometry_data_received = false;
    _gps_data_received = false;
    _attitude_data_received = false;
    _home_position_set = false;

    _current_state = RobotState::IDLE;
    _map_frame_id = "map";
    _gps_frame_id = "gps";
    _base_frame_id = "base_link";
    _max_range = 500.0;

    auto node = shared_from_this();

    // create trajectory generator
    _trajectory_generator = TrajectoryGenerator(_map_frame_id, _max_range);

    // Actions
    _start_navigation_action_server = std::make_shared<StartNavigationServer>(
        node,
        "start_navigation",
        std::bind(&TrajectoryNavigator::handleStartNavigation, this),
        nullptr, std::chrono::milliseconds(500), true);

    // Publishers
    _gps_data_publisher_ = create_publisher<NavSatFix>("gps/fix", rclcpp::QoS(10));
    _odometry_publisher = create_publisher<Odometry>("/odometry", rclcpp::QoS(10));
    _current_trajectory_publisher = create_publisher<Path>("/current_trajectory", rclcpp::QoS(10));
    _vehicle_command_publisher = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", rclcpp::QoS(10));
    _offboard_control_mode_publisher = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", rclcpp::QoS(10));
    _trajectory_setpoint_publisher = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", rclcpp::QoS(10));
    _computed_cmd_vel_publisher = create_publisher<TwistStamped>("/cmd_vel", rclcpp::QoS(10));

    // Subscribers
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    _gps_subscription = create_subscription<SensorGps>(
        "/fmu/out/vehicle_gps_position",
        qos,
        std::bind(&TrajectoryNavigator::gpsCallback, this, std::placeholders::_1));

    _vehicle_status_subscription = create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status",
        qos,
        std::bind(&TrajectoryNavigator::vehicleStatusCallback, this, std::placeholders::_1));

    // _vehicle_local_position_subscription = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    //     "/fmu/out/vehicle_local_position",
    //     qos,
    //     std::bind(&TrajectoryNavigator::localPositionCallback, this, std::placeholders::_1));

    // _vehicle_attitude_subscription = create_subscription<px4_msgs::msg::VehicleAttitude>(
    //     "/fmu/out/vehicle_attitude",
    //     qos,
    //     std::bind(&TrajectoryNavigator::vehicleAttitudeCallback, this, std::placeholders::_1));
    _vehicle_odometry_subscription = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry",
        qos,
        std::bind(&TrajectoryNavigator::vehicleOdometryCallback, this, std::placeholders::_1));

    // Initialize the transform broadcaster
    _tf_broadcaster =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

   

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn TrajectoryNavigator::on_activate(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Activating localization_kf...");

    // Publishers
    _gps_data_publisher_->on_activate();
    _odometry_publisher->on_activate();
    _vehicle_command_publisher->on_activate();
    _offboard_control_mode_publisher->on_activate();
    _trajectory_setpoint_publisher->on_activate();
    _current_trajectory_publisher->on_activate();
    _computed_cmd_vel_publisher->on_activate();

    // Actions
    _start_navigation_action_server->activate();


    createBond();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn TrajectoryNavigator::on_deactivate(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Deactivating localization_kf...");

    // Actions
    _start_navigation_action_server->deactivate();

    // Publishers
    _gps_data_publisher_->on_deactivate();
    _odometry_publisher->on_deactivate();
    _vehicle_command_publisher->on_deactivate();
    _offboard_control_mode_publisher->on_deactivate();
    _trajectory_setpoint_publisher->on_deactivate();
    _current_trajectory_publisher->on_deactivate();
    _computed_cmd_vel_publisher->on_deactivate();

    destroyBond();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn TrajectoryNavigator::on_cleanup(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up localization_kf...");

    // subscriptions
    _gps_subscription.reset();
    _vehicle_status_subscription.reset();
    // _vehicle_local_position_subscription.reset();
    // _vehicle_attitude_subscription.reset();
    _vehicle_odometry_subscription.reset();

    // Actions
    _start_navigation_action_server.reset();

    // Publishers
    _gps_data_publisher_.reset();
    _odometry_publisher.reset();
    _vehicle_command_publisher.reset();
    _offboard_control_mode_publisher.reset();
    _trajectory_setpoint_publisher.reset();
    _current_trajectory_publisher.reset();
    _computed_cmd_vel_publisher.reset();

    // flags
    _gps_data_received = false;
    _odometry_data_received = false;
    _attitude_data_received = false;
    _home_position_set = false;

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn TrajectoryNavigator::on_shutdown(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Shutting down localization_kf...");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  void TrajectoryNavigator::handleStartNavigation()
  {
    RCLCPP_INFO(get_logger(), "Handling start navigation...");

    auto goal = _start_navigation_action_server->get_current_goal();
    if (!goal)
    {
      RCLCPP_ERROR(get_logger(), "No valid goal received for start navigation.");
      return;
    }

    // Declare variables outside the switch statement
    auto result = std::make_shared<StartNavigationAction::Result>();
    auto feedback = std::make_shared<StartNavigationAction::Feedback>();
    std::string trajectory_type_str;
    int trajectory_type;
    Pose ref_pose;
    bool trajectory_finished = false;
    Path path;

    try
    {
      // Simple state machine implementation where we wait for trajectory to finish before
      // performing the next request
      while (rclcpp::ok() && !trajectory_finished)
      {
        // Check for preemption
        if (_start_navigation_action_server->is_preempt_requested())
        {
          if (_current_state != RobotState::IDLE)
          {
            feedback->status = "Currently navigating";
            _start_navigation_action_server->publish_feedback(feedback);
          }
        }

        // TODO, if this gets too complex, maybe consider a bt :()
        switch (_current_state)
        {
        case RobotState::IDLE:

            // set the home position
            setHomePosition();
          _current_state = RobotState::PRE_NAVIGATING;
          break;
        case RobotState::PRE_NAVIGATING:

          // arm
          feedback->status = "Arming...";
          _start_navigation_action_server->publish_feedback(feedback);
          waitForArm();
          feedback->status = "Armed";
          _start_navigation_action_server->publish_feedback(feedback);

          // takeoff
          feedback->status = "Taking off...";
          _start_navigation_action_server->publish_feedback(feedback);
          waitForTakeoff();
          feedback->status = "Taking off completed";
          _start_navigation_action_server->publish_feedback(feedback);

          // wait for offboard mode
          feedback->status = "Enabling offboard mode...";
          _start_navigation_action_server->publish_feedback(feedback);
          waitForOffboardMode();
          feedback->status = "Offboard mode enabled";
          _start_navigation_action_server->publish_feedback(feedback);

          feedback->status = "Starting navigation...";
          _start_navigation_action_server->publish_feedback(feedback);
          _current_state = RobotState::NAVIGATING;

          break;

        case RobotState::NAVIGATING:

          // Navigate
          trajectory_type = static_cast<int>(goal->trajectory);
          trajectory_type_str = get_trajectory_type_string(trajectory_type);
          RCLCPP_INFO(get_logger(), "Starting navigation with trajectory type: %s", trajectory_type_str.c_str());

          // set ref pose to home pose
          ref_pose = _home_vehicle_odometry.pose.pose;
          // use current z
          ref_pose.position.z = _cruise_height;

          path = _trajectory_generator.generateTrajectory(trajectory_type, ref_pose);
          feedback->status = "Navigating...";
          _start_navigation_action_server->publish_feedback(feedback);

          // perform navigation
          _current_trajectory_publisher->publish(path);
          goToTrajectory(path);
          feedback->status = "Navigation completed";
          _start_navigation_action_server->publish_feedback(feedback);

          _current_state = RobotState::LANDING;
          break;

        case RobotState::LANDING:

          feedback->status = "Landing...";
          _start_navigation_action_server->publish_feedback(feedback);
          Land();
          feedback->status = "Landing completed";
          _start_navigation_action_server->publish_feedback(feedback);

          feedback->status = "Disarming...";
          _start_navigation_action_server->publish_feedback(feedback);
          waitForDisarm();
          feedback->status = "Disarmed";
          _start_navigation_action_server->publish_feedback(feedback);

          // Switch back to hold mode
          switchToHoldMode();

          // finish for now
          // Simulate success for now
          result->success = true;
          result->message = "Navigation started successfully.";
          _start_navigation_action_server->succeeded_current(result);
          _current_state = RobotState::IDLE;
          trajectory_finished = true;
          break;
        default:
          // Handle other states if necessary
          break;
        }
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "Exception while handling start navigation: %s", e.what());
      result->success = false;
      _start_navigation_action_server->terminate_current();
    }
  }

  void TrajectoryNavigator::gpsCallback(const SensorGps::SharedPtr msg)
  {
    RCLCPP_INFO_STREAM_ONCE(get_logger(), "GPS data received");
    // Header
    _current_gps_data.header.stamp = rclcpp::Time(msg->timestamp * 1000); // Convert microseconds to nanoseconds
    _current_gps_data.header.frame_id = _gps_frame_id;

    // Status
    _current_gps_data.status.status = (msg->fix_type >= 3) ? sensor_msgs::msg::NavSatStatus::STATUS_FIX : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    _current_gps_data.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    // Position
    _current_gps_data.latitude = msg->lat * 1e-7;  // Convert to degrees
    _current_gps_data.longitude = msg->lon * 1e-7; // Convert to degrees
    _current_gps_data.altitude = msg->alt * 1e-3;  // Convert to meters

    // Covariance
    _current_gps_data.position_covariance[0] = msg->eph * msg->eph; // Variance in East direction
    _current_gps_data.position_covariance[4] = msg->eph * msg->eph; // Variance in North direction
    _current_gps_data.position_covariance[8] = msg->epv * msg->epv; // Variance in Up direction
    _current_gps_data.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

    // Publish the converted message
    _gps_data_publisher_->publish(_current_gps_data);
    _gps_data_received = true;
  }

  void TrajectoryNavigator::vehicleStatusCallback(const VehicleStatus::SharedPtr msg)
  {
    _current_vehicle_status = *msg;
  }


  void TrajectoryNavigator::vehicleOdometryCallback(const VehicleOdometry::SharedPtr msg)
  {

    Odometry odom_enu;

    // Header
    odom_enu.header.stamp = rclcpp::Time(msg->timestamp * 1000); // Convert microseconds to nanoseconds
    odom_enu.header.frame_id = _map_frame_id;
    odom_enu.child_frame_id = _base_frame_id;

    if (msg->pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED)
    {
      // Just to make sure for now, maybe look into this later
      RCLCPP_INFO_THROTTLE(
          get_logger(), *this->get_clock(), 1000,
           "Only NED frame is supported");
      return;
    }

    // Position
    Eigen::Vector3d ned_position(msg->position[0], msg->position[1], msg->position[2]);
    Eigen::Vector3d enu_position = px4_ros_com::frame_transforms::ned_to_enu_local_frame(ned_position);
    odom_enu.pose.pose.position.x = enu_position.x();
    odom_enu.pose.pose.position.y = enu_position.y();
    odom_enu.pose.pose.position.z = enu_position.z();

    // Orientation
    Eigen::Quaterniond ned_orientation;
    ned_orientation.x() = msg->q[1];
    ned_orientation.y() = msg->q[2];
    ned_orientation.z() = msg->q[3];
    ned_orientation.w() = msg->q[0];

    Eigen::Quaterniond enu_orientation = px4_ros_com::frame_transforms::px4_to_ros_orientation(ned_orientation);
    odom_enu.pose.pose.orientation.x = enu_orientation.x();
    odom_enu.pose.pose.orientation.y = enu_orientation.y();
    odom_enu.pose.pose.orientation.z = enu_orientation.z();
    odom_enu.pose.pose.orientation.w = enu_orientation.w();

    // Velocity
    if (msg->velocity_frame != px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED)
    {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *this->get_clock(), 1000,
          "Only NED frame is supported");
      return;
    }
    // Convert velocity from NED to ENU
    Eigen::Vector3d ned_velocity(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
    // frd vel
    Eigen::Vector3d vel_frd = px4_ros_com::frame_transforms::ned_to_aircraft_frame(ned_velocity, ned_orientation);
    // flu vel Ros standard
    Eigen::Vector3d vel_flu = px4_ros_com::frame_transforms::aircraft_to_baselink_body_frame(vel_frd);
    odom_enu.twist.twist.linear.x = vel_flu.x();
    odom_enu.twist.twist.linear.y = vel_flu.y();
    odom_enu.twist.twist.linear.z = vel_flu.z();

    // angular velocity
    // FRD -> FLU
    Eigen::Vector3d frd_angular_velocity(msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2]);
    Eigen::Vector3d flu_angular_velocity = px4_ros_com::frame_transforms::aircraft_to_baselink_body_frame(frd_angular_velocity);
    odom_enu.twist.twist.angular.x = flu_angular_velocity.x();
    odom_enu.twist.twist.angular.y = flu_angular_velocity.y();
    odom_enu.twist.twist.angular.z = flu_angular_velocity.z();



    if (_odometry_data_received)
    {
      // We want to fake compute the computed_cmd_vel as the command
      TwistStamped cmd_vel;
      cmd_vel.header.stamp = odom_enu.header.stamp;
      cmd_vel.header.frame_id = _base_frame_id;
      // just copy the velocity for now
      cmd_vel.twist = odom_enu.twist.twist;
      _computed_cmd_vel_publisher->publish(cmd_vel);
      
    }


    // Covariances TODO
   _current_vehicle_odometry = odom_enu;

    // Both odometry and vehicle attitude data received
    // Publish the converted message
    _odometry_publisher->publish(odom_enu);

    // Publish TF from base_link to map
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = odom_enu.header.stamp;
    transform_stamped.header.frame_id = _map_frame_id;
    transform_stamped.child_frame_id = _base_frame_id;

    transform_stamped.transform.translation.x = odom_enu.pose.pose.position.x;
    transform_stamped.transform.translation.y = odom_enu.pose.pose.position.y;
    transform_stamped.transform.translation.z = odom_enu.pose.pose.position.z;

    transform_stamped.transform.rotation.x = odom_enu.pose.pose.orientation.x;
    transform_stamped.transform.rotation.y = odom_enu.pose.pose.orientation.y;
    transform_stamped.transform.rotation.z = odom_enu.pose.pose.orientation.z;
    transform_stamped.transform.rotation.w = odom_enu.pose.pose.orientation.w;

    _tf_broadcaster->sendTransform(transform_stamped);

    _odometry_data_received = true;


    // We want to fake compute the computed_cmd_vel

  }

  // void TrajectoryNavigator::localPositionCallback(const VehicleLocalPosition::SharedPtr msg)
  // {
  //   nav_msgs::msg::Odometry odometry_msg;

  //   if (!_attitude_data_received)
  //   {
  //     return;
  //   }
  //   RCLCPP_INFO_STREAM_ONCE(get_logger(), "Odometry data received");


  //   // Header
  //   odometry_msg.header.stamp = this->get_clock()->now();
  //   odometry_msg.header.frame_id = _map_frame_id;
  //   odometry_msg.child_frame_id = _base_frame_id;

  //   // Position
  //   Eigen::Vector3d ned_position(msg->x, msg->y, msg->z);
  //   Eigen::Vector3d enu_position = px4_ros_com::frame_transforms::ned_to_enu_local_frame(ned_position);
  //   odometry_msg.pose.pose.position.x = enu_position.x();
  //   odometry_msg.pose.pose.position.y = enu_position.y();
  //   odometry_msg.pose.pose.position.z = enu_position.z();

  //   // Orientation
  //   Eigen::Quaterniond ned_orientation;
  //   ned_orientation.x() = _current_vehicle_attitude.q[1];
  //   ned_orientation.y() = _current_vehicle_attitude.q[2];
  //   ned_orientation.z() = _current_vehicle_attitude.q[3];
  //   ned_orientation.w() = _current_vehicle_attitude.q[0];

  //   Eigen::Quaterniond enu_orientation = px4_ros_com::frame_transforms::px4_to_ros_orientation(ned_orientation);
  //   odometry_msg.pose.pose.orientation.x = enu_orientation.x();
  //   odometry_msg.pose.pose.orientation.y = enu_orientation.y();
  //   odometry_msg.pose.pose.orientation.z = enu_orientation.z();
  //   odometry_msg.pose.pose.orientation.w = enu_orientation.w();

  //   // TODO: velocity and covariance

  //   _current_vehicle_odometry = odometry_msg;

  //   // Both odometry and vehicle attitude data received
  //   // Publish the converted message
  //   _odometry_publisher->publish(odometry_msg);

  //   // Publish TF from base_link to map
  //   geometry_msgs::msg::TransformStamped transform_stamped;
  //   transform_stamped.header.stamp = odometry_msg.header.stamp;
  //   transform_stamped.header.frame_id = _map_frame_id;
  //   transform_stamped.child_frame_id = _base_frame_id;

  //   transform_stamped.transform.translation.x = odometry_msg.pose.pose.position.x;
  //   transform_stamped.transform.translation.y = odometry_msg.pose.pose.position.y;
  //   transform_stamped.transform.translation.z = odometry_msg.pose.pose.position.z;

  //   transform_stamped.transform.rotation.x = odometry_msg.pose.pose.orientation.x;
  //   transform_stamped.transform.rotation.y = odometry_msg.pose.pose.orientation.y;
  //   transform_stamped.transform.rotation.z = odometry_msg.pose.pose.orientation.z;
  //   transform_stamped.transform.rotation.w = odometry_msg.pose.pose.orientation.w;

  //   _tf_broadcaster->sendTransform(transform_stamped);
  //   _odometry_data_received = true;
  // }

  // void TrajectoryNavigator::vehicleAttitudeCallback(const VehicleAttitude::SharedPtr msg)
  // {
  //   _current_vehicle_attitude = *msg;
  //   _attitude_data_received = true;
  // }

  void TrajectoryNavigator::setHomePosition()
  {
    while (rclcpp::ok())
    {
      if (_gps_data_received && _odometry_data_received)
      {
        _home_gps_position = _current_gps_data;
        _home_vehicle_odometry = _current_vehicle_odometry;
        _home_position_set = true;
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      RCLCPP_INFO_THROTTLE(
          get_logger(), *this->get_clock(), 1000,
          "Waiting for GPS and odometry data to set home position. GPS received: %s, Odometry received: %s",
          _gps_data_received ? "true" : "false",
          _odometry_data_received ? "true" : "false");
    }
  }

  void TrajectoryNavigator::publishVehicleCommand(uint16_t command, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
  {
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
    msg.param6 = param6;
    msg.param7 = param7;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    _vehicle_command_publisher->publish(msg);
  }

  void TrajectoryNavigator::waitForArm()
  {
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    while (!hasTimedOut(start, _default_timeout) && rclcpp::ok())
    {
      // Check if the drone is armed
      if (_current_vehicle_status.arming_state == VehicleStatus::ARMING_STATE_ARMED)
      {
        RCLCPP_INFO(get_logger(), "Drone is armed.");
        return;
      }
      // send arm command
      publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_WARN(get_logger(), "Timed out waiting for drone to arm.");
    throw std::runtime_error("Drone did not arm within timeout");
  }

  void TrajectoryNavigator::waitForDisarm()
  {
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    while (!hasTimedOut(start, _default_timeout) && rclcpp::ok())
    {
      // Check if the drone is disarmed
      if (_current_vehicle_status.arming_state == VehicleStatus::ARMING_STATE_STANDBY)
      {
        RCLCPP_INFO(get_logger(), "Drone is disarmed.");
        return;
      }

      // send disarm command
      publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0);

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_WARN(get_logger(), "Timed out waiting for drone to disarm.");
    throw std::runtime_error("Drone did not disarm within timeout");
  }

  void TrajectoryNavigator::waitForOffboardMode()
  {
    for (int i = 0; i < 20; i++)
    {
      setOffboardControlMode();
      publishTrajectorySetpoint(
          0.0,
          0.0,
          0.0,
          0.0);

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
  }

  void TrajectoryNavigator::setOffboardControlMode()
  {
    // position control mode
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    _offboard_control_mode_publisher->publish(msg);
  }

  void TrajectoryNavigator::publishTrajectorySetpoint(float x, float y, float z, float yaw)
  {
    // Publish the trajectory setpoint
    TrajectorySetpoint msg{};
    msg.position = {x, y, z};

    // prevent using the feed forward terms
    msg.velocity[0] = std::numeric_limits<float>::quiet_NaN();
    msg.velocity[1] = std::numeric_limits<float>::quiet_NaN();
    msg.velocity[2] = std::numeric_limits<float>::quiet_NaN();
    msg.yawspeed = std::numeric_limits<float>::quiet_NaN();

    msg.acceleration[0] = std::numeric_limits<float>::quiet_NaN();
    msg.acceleration[1] = std::numeric_limits<float>::quiet_NaN();
    msg.acceleration[2] = std::numeric_limits<float>::quiet_NaN();
    msg.yaw = yaw;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    _trajectory_setpoint_publisher->publish(msg);
  }

  void TrajectoryNavigator::waitForTakeoff()
  {
    RCLCPP_INFO(get_logger(), "Waiting for takeoff...");

    float ref_altitude = _current_gps_data.altitude;
    float takeoff_altitude = ref_altitude + _cruise_height;

    publishVehicleCommand(
        VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF,
        0.1f, // minimum pitch
        0,
        0,
        1.57,
        _current_gps_data.latitude,
        _current_gps_data.longitude,
        takeoff_altitude);

    auto hasTakenOff = [&]() {
        return std::abs(_current_gps_data.altitude - takeoff_altitude) < 1.0f;
    };

    auto start = std::chrono::steady_clock::now();
    while (!hasTakenOff())
    {
      if (hasTimedOut(start, 200.0f))
      {
        RCLCPP_WARN(get_logger(), "Timed out waiting for takeoff.");
        throw std::runtime_error("Takeoff did not occur within timeout");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(get_logger(), "Takeoff successful.");
  }

  void TrajectoryNavigator::Land()
  {

    publishVehicleCommand(
        VehicleCommand::VEHICLE_CMD_NAV_LAND,
        0.1f, // minimum pitch
        0,
        0,
        1.57,
        _current_gps_data.latitude,
        _current_gps_data.longitude);
  }

  bool TrajectoryNavigator::reachedGoal(const Pose &set_pose)
  {
    // check the distance between set pose and current odom
    float distance = std::hypot(set_pose.position.x - _current_vehicle_odometry.pose.pose.position.x,
                                set_pose.position.y - _current_vehicle_odometry.pose.pose.position.y);

    double yaw_set = tf2::getYaw(set_pose.orientation);
    double yaw_current = tf2::getYaw(_current_vehicle_odometry.pose.pose.orientation);

    double yaw_diff = angles::normalize_angle(yaw_set - yaw_current);
    double yaw_error = std::abs(yaw_diff);
    if (distance < 0.5f && yaw_error < 0.1f)
    {
      RCLCPP_INFO(get_logger(), "Reached goal: [%f, %f, %f]", set_pose.position.x, set_pose.position.y, set_pose.position.z);
      return true;
    }
    return false;
  }

  void TrajectoryNavigator::goToTrajectory(const Path &path)
  {

    for (const auto &pose : path.poses)
    {
      RCLCPP_INFO(get_logger(), "Navigating to point: [%f, %f, %f, %f]", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, tf2::getYaw(pose.pose.orientation));
      while (true)
      {

        if (reachedGoal(pose.pose))
        {
          RCLCPP_INFO(get_logger(), "Reached point: [%f, %f, %f, %f]", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, tf2::getYaw(pose.pose.orientation));
          break;
        }
        
        Eigen::Quaterniond enu_quaternion;
        enu_quaternion.x() = pose.pose.orientation.x;
        enu_quaternion.y() = pose.pose.orientation.y;
        enu_quaternion.z() = pose.pose.orientation.z;
        enu_quaternion.w() = pose.pose.orientation.w;

        Eigen::Quaterniond ned_quaternion = px4_ros_com::frame_transforms::ros_to_px4_orientation(enu_quaternion);

        tf2::Quaternion tf2_ned_quaternion;
        tf2_ned_quaternion.setX(ned_quaternion.x());
        tf2_ned_quaternion.setY(ned_quaternion.y());
        tf2_ned_quaternion.setZ(ned_quaternion.z());
        tf2_ned_quaternion.setW(ned_quaternion.w());

        // get yaw from ned quaternion
        double ned_yaw = tf2::getYaw(tf2_ned_quaternion);
        setOffboardControlMode();
        publishTrajectorySetpoint(pose.pose.position.y, pose.pose.position.x, -pose.pose.position.z, ned_yaw);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }

  std::string TrajectoryNavigator::get_trajectory_type_string(int type)
  {
    switch (type)
    {
    case 0:
      return "Square";
    case 1:
      return "Circle";
    case 2:
      return "FigureEight";
    default:
      return "Unknown";
    }
  }

  void TrajectoryNavigator::switchToHoldMode()
  {
  RCLCPP_INFO(get_logger(), "Switching back to hold mode...");
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    // while(_current_vehicle_status.nav_state != VehicleStatus::NAVIGATION_STATE_AUTO_LOITER)
    // {
      // if (hasTimedOut(start, 200.0f))
      // {
      //   RCLCPP_WARN(get_logger(), "Timed out waiting for hold mode.");
      //   throw std::runtime_error("Hold mode did not occur within timeout");
      // }
      publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 2);
  //   }
  }

} // namespace localization_kf_navigator

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(localization_kf_navigator::TrajectoryNavigator)
