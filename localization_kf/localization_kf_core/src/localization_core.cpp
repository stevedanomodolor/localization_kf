#include <localization_kf_core/localization_core.hpp>

namespace localization_kf_core {

// Use GRAVITY constant from EKF header
// stati    // Variables
constexpr double GRAVITY = 9.81;

LocalizationCore::LocalizationCore(const rclcpp::NodeOptions &options)
    : nav2_util::LifecycleNode("localization_core", "", options), _sensor_timeout(0, 0) {
    // declare parameters
    this->declare_parameter<double>("frequency", 30.0);
    this->declare_parameter<std::vector<double>>("process_noise_cov", std::vector<double>{});
    this->declare_parameter<std::vector<double>>("gps_stddev", std::vector<double>{0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("magnetic_field_stddev", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("altimeter_stddev", std::vector<double>{0.0, 0.0});
    this->declare_parameter<double>("sensor_timeout", 0.1);
    this->declare_parameter<double>("base_offset", 0.22);
}

nav2_util::CallbackReturn LocalizationCore::on_configure(const rclcpp_lifecycle::State &state) {
    RCLCPP_INFO(get_logger(), "Configuring localization_core...");

    // initialize status flags
    _gps_data_received = false;
    _imu_data_received = false;
    _magnetic_field_data_received = false;
    _altimeter_data_received = false;
    _odometry_ground_truth_data_received = false;
    _initial_home_state_set = false;

    // get parameters
    this->get_parameter("frequency", _frequency);
    if (_frequency <= 0.0) {
        RCLCPP_WARN(get_logger(), "Frequency must be positive. Setting to default 30 Hz.");
        _frequency = 30.0;
    }

    // Get control input configuration
    this->get_parameter("use_imu_as_control", _use_imu_as_control);
    this->get_parameter("use_imu_as_measurement", _use_imu_as_measurement);
    
    RCLCPP_INFO(get_logger(), "IMU configuration: control=%s, measurement=%s", 
                _use_imu_as_control ? "enabled" : "disabled",
                _use_imu_as_measurement ? "enabled" : "disabled");

    // Process noise
    std::vector<double> process_noise_cov;
    this->get_parameter("process_noise_cov", process_noise_cov);
    // process noise should have 12 x 12 elements
    if (process_noise_cov.size() != (EKF::STATE_SIZE * EKF::STATE_SIZE)) {
        RCLCPP_WARN(get_logger(), "Process noise covariance parameter must have %d elements. Got %zu elements.",
                    EKF::STATE_SIZE * EKF::STATE_SIZE, process_noise_cov.size());
        throw std::runtime_error("Process noise covariance parameter has incorrect size");
    }

    std::vector<double> gps_stddev;
    this->get_parameter("gps_stddev", gps_stddev);
    if (gps_stddev.size() != 4) {
        RCLCPP_WARN(get_logger(), "GPS stddev parameter must have 4 elements. Setting to "
                                  "default [0.0, 0.0, 0.0, 0.0].");
        _R_gps = Eigen::MatrixXd::Zero(6, 6);
    } else {
        _R_gps = Eigen::MatrixXd::Zero(6, 6);
        // convert horizontal stddev from degrees to meters
        gps_stddev[0] = gps_stddev[0] * 111133;
        _R_gps(0, 0) = gps_stddev[0] * gps_stddev[0]; // x
        _R_gps(1, 1) = gps_stddev[0] * gps_stddev[0]; // y
        // vertical stddev is already in meters
        _R_gps(2, 2) = gps_stddev[1] * gps_stddev[1]; // z
        // velocity stddev is already in m/s
        _R_gps(3, 3) = gps_stddev[2] * gps_stddev[2]; // vx
        _R_gps(4, 4) = gps_stddev[2] * gps_stddev[2]; // vy
        _R_gps(5, 5) = gps_stddev[3] * gps_stddev[3]; // vz
    }

    this->get_parameter("magnetic_field_stddev", _magnetic_field_stddev);
    if (_magnetic_field_stddev.size() != 3) {
        RCLCPP_WARN(get_logger(), "Magnetic field stddev parameter must have 3 elements. "
                                  "Setting to default [0.0, 0.0, 0.0].");
        _magnetic_field_stddev = std::vector<double>{0.0, 0.0, 0.0};
    }

    // Altimeter stddev
    std::vector<double> altimeter_stddev;
    this->get_parameter("altimeter_stddev", altimeter_stddev);
    if (altimeter_stddev.size() != 2) {
        RCLCPP_WARN(get_logger(), "Altimeter stddev parameter must have 2 elements. Setting to default [0.0, 0.0].");
        _R_altimeter = Eigen::MatrixXd::Zero(2, 2);
    } else {
        _R_altimeter = Eigen::MatrixXd::Zero(2, 2);
        _R_altimeter(0, 0) = altimeter_stddev[0] * altimeter_stddev[0]; // vertical altitude
        _R_altimeter(1, 1) = altimeter_stddev[1] * altimeter_stddev[1]; // vertical velocity
    }

    double sensor_timeout_sec;
    this->get_parameter("sensor_timeout", sensor_timeout_sec);
    if (sensor_timeout_sec <= 0.0) {
        RCLCPP_WARN(get_logger(), "Sensor timeout must be positive. Setting to default 0.1 s.");
        sensor_timeout_sec = 0.1;
    }
    _sensor_timeout = rclcpp::Duration::from_seconds(sensor_timeout_sec);

    this->get_parameter("base_offset", _base_offset);

    // Variables
    _filter = std::make_shared<EKF>(_sensor_timeout, process_noise_cov);
    // publishers
    _odometry_estimated_pub = this->create_publisher<Odometry>("/odometry/estimated", 1);
    _acceleration_estimated_pub = this->create_publisher<AccelWithCovarianceStamped>("/acceleration/estimated", 1);
    _odometry_estimate_error_pub = this->create_publisher<Odometry>("/odometry/estimate_error", 1);

    // subscribers
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    _gps_fix_sub = this->create_subscription<GpsFix>(
        "/gps/fix", qos, std::bind(&LocalizationCore::gpsFixCallback, this, std::placeholders::_1));

    _imu_sub = this->create_subscription<Imu>("/imu/data", qos,
                                              std::bind(&LocalizationCore::imuCallback, this, std::placeholders::_1));

    _magnetic_field_sub = this->create_subscription<MagneticField>(
        "/magnetometer/data", qos, std::bind(&LocalizationCore::magneticFieldCallback, this, std::placeholders::_1));

    _altimeter_sub = this->create_subscription<Altimeter>(
        "/altimeter/data", qos, std::bind(&LocalizationCore::altimeterCallback, this, std::placeholders::_1));

    _odometry_ground_truth_sub = this->create_subscription<Odometry>(
        "/odom_ground_truth", qos,
        std::bind(&LocalizationCore::odometryGroundTruthCallback, this, std::placeholders::_1));

    // create timer
    _timer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / _frequency)),
                                     std::bind(&LocalizationCore::timerCallback, this));

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn LocalizationCore::on_activate(const rclcpp_lifecycle::State &state) {
    RCLCPP_INFO(get_logger(), "Activating localization_core...");
    // publishers
    _odometry_estimated_pub->on_activate();
    _acceleration_estimated_pub->on_activate();
    _odometry_estimate_error_pub->on_activate();

    createBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn LocalizationCore::on_deactivate(const rclcpp_lifecycle::State &state) {
    RCLCPP_INFO(get_logger(), "Deactivating localization_core...");

    // publishers
    _odometry_estimated_pub->on_deactivate();
    _acceleration_estimated_pub->on_deactivate();
    _odometry_estimate_error_pub->on_deactivate();

    destroyBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn LocalizationCore::on_cleanup(const rclcpp_lifecycle::State &state) {
    RCLCPP_INFO(get_logger(), "Cleaning up localization_core...");

    // timer
    if (_timer) {
        _timer->cancel();
        _timer.reset();
    }

    // subscribers
    _gps_fix_sub.reset();
    _imu_sub.reset();
    _magnetic_field_sub.reset();
    _altimeter_sub.reset();
    _odometry_ground_truth_sub.reset();

    // publishers
    _odometry_estimated_pub.reset();
    _acceleration_estimated_pub.reset();
    _odometry_estimate_error_pub.reset();

    // reset status flags
    _gps_data_received = false;
    _imu_data_received = false;
    _magnetic_field_data_received = false;
    _altimeter_data_received = false;
    _odometry_ground_truth_data_received = false;
    _initial_home_state_set = false;

    _filter.reset();
    _last_message_times.clear();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn LocalizationCore::on_shutdown(const rclcpp_lifecycle::State &state) {
    RCLCPP_INFO(get_logger(), "Shutting down localization_core...");
    return nav2_util::CallbackReturn::SUCCESS;
}

void LocalizationCore::timerCallback() {

    // Early exit if initial position not set
    if (!is_initial_position_set()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000,
        "Initial position not set, skipping timer callback");
        return;
    }
    
    rclcpp::Time current_time = this->get_clock()->now();
    _filter->processControlBuffer(current_time);
    _filter->processMeasurementBuffer(current_time);

    // Always publish odometry regardless of measurement availability
    Odometry odom_msg;
    if (_filter->getOdometry(odom_msg)) {
        _odometry_estimated_pub->publish(odom_msg);

        // compute and publish odometry estimate error if ground truth available
        if (_odometry_ground_truth_data_received) {
            Odometry error_msg;
            computeOdometryError(odom_msg, *_latest_sensor_data.odom, error_msg);
            _odometry_estimate_error_pub->publish(error_msg);
        }
    }
    
    // Always publish acceleration
    AccelWithCovarianceStamped acc_msg;
    if (_filter->getAcceleration(acc_msg)) {
        _acceleration_estimated_pub->publish(acc_msg);
    }
}

bool LocalizationCore::is_initial_position_set() {
    // early exit
    if (_initial_home_state_set) {
        return true;
    }

    if (!_gps_data_received || !_altimeter_data_received || !_magnetic_field_data_received ||
        !good_gps(*_latest_sensor_data.gps)) {
        return false;
    }

    // Set initial reference state
    // Robot boot up location is home position? maybe allow user to set from
    // service?
    _ref_local_cartesian = GeographicLib::LocalCartesian(
        _latest_sensor_data.gps->latitude, _latest_sensor_data.gps->longitude, _latest_sensor_data.gps->altitude);

    _ref_altitude_altimeter = _latest_sensor_data.altimeter->vertical_position;
    _ref_gps_altitude = _latest_sensor_data.gps->altitude;

    RCLCPP_INFO(get_logger(), "Reference point set to GPS[lat, lon, alt]: [%f, %f, %f]",
                _latest_sensor_data.gps->latitude, _latest_sensor_data.gps->longitude, _latest_sensor_data.gps->altitude);

    // build initial state
    // use gps state as initial state definition
    // Robot is stationary at startup
    Eigen::VectorXd initial_state(EKF::STATE_SIZE);
    initial_state.setZero();
    // set orientation heading from magnetic field reading
    initial_state(8) = _latest_sensor_data.gps->track * M_PI / 180.0; // degrees to radians

    // build initial covariance
    Eigen::MatrixXd initial_covariance = Eigen::MatrixXd::Zero(EKF::STATE_SIZE, EKF::STATE_SIZE);
    // position covariance - reasonable initial uncertainty for measurement-only mode
    initial_covariance(0, 0) = 0.01; // x - 1m uncertainty
    initial_covariance(1, 1) = 0.01; // y - 1m uncertainty
    initial_covariance(2, 2) = 0.01; // z - 1m uncertainty
    // velocity covariance
    initial_covariance(3, 3) = 0.5; // vx - 0.5 m/s uncertainty
    initial_covariance(4, 4) = 0.5; // vy - 0.5 m/s uncertainty
    initial_covariance(5, 5) = 0.5; // vz - 0.5 m/s uncertainty
    // orientation covariance
    initial_covariance(6, 6) = 0.1; // roll - ~6 deg uncertainty
    initial_covariance(7, 7) = 0.1; // pitch - ~6 deg uncertainty
    initial_covariance(8, 8) = 0.2; // yaw - ~11 deg uncertainty (will be overridden by mag)
    // angular velocity covariance  
    initial_covariance(9, 9) = 0.1; // p - 0.1 rad/s uncertainty
    initial_covariance(10, 10) = 0.1; // q - 0.1 rad/s uncertainty
    initial_covariance(11, 11) = 0.1; // r - 0.1 rad/s uncertainty

    // orientation covariance
    // compute magnetic heading covariance
    double mag_heading_variance = computeMagHeadingCovariance(_magnetic_field_stddev, *_latest_sensor_data.mag);
    initial_covariance(8, 8) = mag_heading_variance; // yaw

    _filter->initialize(initial_state, initial_covariance);

    _initial_home_state_set = true;

    // print state
    RCLCPP_INFO(get_logger(),
                "Initial home state set, Gps[lat, lon, alt]: [%f, %f, %f], "
                "Heading[deg]: %f, Altimeter[alt]: %f",
                _latest_sensor_data.gps->latitude, _latest_sensor_data.gps->longitude,
                _latest_sensor_data.gps->altitude,
                initial_state(8) * 180.0 / M_PI, // radians to degrees
                _latest_sensor_data.altimeter->vertical_position);
    return true;
}

// subscriber callbacks
void LocalizationCore::gpsFixCallback(const GpsFix::SharedPtr msg) {
    RCLCPP_INFO_STREAM_ONCE(get_logger(), "GPS fix data received");

    // Message timing tracking
    if (_last_message_times.count("gps") == 0) {
        _last_message_times.insert(std::make_pair("gps", rclcpp::Time(msg->header.stamp)));
    }

    rclcpp::Time msg_time = rclcpp::Time(msg->header.stamp);
    // handle old messages
    if (msg_time < _last_message_times["gps"]) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "GPS message received with non-increasing timestamp. Ignoring this message.");
        return;
    }
    _last_message_times["gps"] = msg_time;

    // Store GPS data for initialization purposes
    _latest_sensor_data.gps = msg;
    _gps_data_received = true;

    // Only process GPS measurements after initial home state is set
    if (!_initial_home_state_set) {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *this->get_clock(), 1000,
                             "GPS data received but initial home state not set yet. Storing for initialization.");
        return;
    }

    Eigen::VectorXd measurement(6);
    // convert lat/lon to local cartesian coordinates
    double x, y, z;
    _ref_local_cartesian.Forward(msg->latitude, msg->longitude, msg->altitude, x, y, z);
    
    measurement(0) = x;
    measurement(1) = y;
    measurement(2) = msg->altitude - _ref_gps_altitude;
    measurement(3) = msg->speed * cos(msg->track * M_PI / 180.0); // vx, convert degrees to radians
    measurement(4) = msg->speed * sin(msg->track * M_PI / 180.0); // vy
    measurement(5) = msg->climb;    
    Eigen::MatrixXd covariance(6, 6);
    covariance = _R_gps;

    auto measurement_ptr =
        std::make_shared<Measurement>(MeasurementType::GPS, rclcpp::Time(msg->header.stamp), measurement, covariance);
    _filter->addMeasurement(measurement_ptr);
}

void LocalizationCore::imuCallback(const Imu::SharedPtr msg) {
    RCLCPP_INFO_STREAM_ONCE(get_logger(), "IMU data received");

    // Message timing tracking
    if (_last_message_times.count("imu") == 0) {
        _last_message_times.insert(std::make_pair("imu", rclcpp::Time(msg->header.stamp)));
    }
    rclcpp::Time msg_time = rclcpp::Time(msg->header.stamp);
    // handle old messages
    if (msg_time < _last_message_times["imu"]) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "IMU message received with non-increasing timestamp. Ignoring this message.");
        return;
    }
    _last_message_times["imu"] = msg_time;

    _latest_sensor_data.imu = msg;

    // Only process IMU measurements after initial home state is set
    if (!_initial_home_state_set) {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *this->get_clock(), 1000,
                             "IMU data received but initial home state not set yet. Storing for initialization.");
        return;
    }

    Eigen::VectorXd control_input(EKF::CONTROL_SIZE);

   
    // Use filtered IMU data for control input
    control_input(0) = msg->linear_acceleration.x;
    control_input(1) = msg->linear_acceleration.y;
    control_input(2) = msg->linear_acceleration.z;
    control_input(3) = msg->angular_velocity.x;
    control_input(4) = msg->angular_velocity.y;
    control_input(5) = msg->angular_velocity.z;

    // # Gravity compensation
    Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    Eigen::Vector3d gravity_world(0, 0, GRAVITY);  // Gravity effect: +9.81 upward reaction
    Eigen::Vector3d gravity_effect_body = q.inverse() * gravity_world;
    control_input.head<3>() -= gravity_effect_body;
    auto control_input_ptr = std::make_shared<ControlInput>(rclcpp::Time(msg->header.stamp), control_input);
    _filter->addControlInput(control_input_ptr);

   
    _imu_data_received = true;
}

void LocalizationCore::magneticFieldCallback(const MagneticField::SharedPtr msg) {
    RCLCPP_INFO_STREAM_ONCE(get_logger(), "Magnetic field data received");

    // Message timing tracking
    if (_last_message_times.count("magnetic_field") == 0) {
        _last_message_times.insert(std::make_pair("magnetic_field", rclcpp::Time(msg->header.stamp)));
    }

    rclcpp::Time msg_time = rclcpp::Time(msg->header.stamp);
    // handle old messages
    if (msg_time < _last_message_times["magnetic_field"]) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Magnetic field message received with non-increasing timestamp. Ignoring this message.");
        return;
    }
    _last_message_times["magnetic_field"] = msg_time;
    _latest_sensor_data.mag = msg;
    _magnetic_field_data_received = true;

    // Only process magnetometer measurements after initial home state is set
    if (!_initial_home_state_set) {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *this->get_clock(), 1000,
                             "Magnetometer data received but initial home state not set yet. Storing for initialization.");
        return;
    }

    // compute yaw and covariance
    double mag_heading = computeMagHeading(*msg);
    double mag_heading_variance = computeMagHeadingCovariance(_magnetic_field_stddev, *msg);
    Eigen::VectorXd measurement(1);
    measurement(0) = mag_heading;
    Eigen::MatrixXd covariance(1, 1);
    covariance(0, 0) = mag_heading_variance;

    auto measurement_ptr = std::make_shared<Measurement>(MeasurementType::MAGNETOMETER, rclcpp::Time(msg->header.stamp),
                                                         measurement, covariance);
    _filter->addMeasurement(measurement_ptr);
}

void LocalizationCore::altimeterCallback(const Altimeter::SharedPtr msg) {
    RCLCPP_INFO_STREAM_ONCE(get_logger(), "Altimeter data received");

    // Message timing tracking
    if (_last_message_times.count("altimeter") == 0) {
        _last_message_times.insert(std::make_pair("altimeter", rclcpp::Time(msg->header.stamp)));
    }

    rclcpp::Time msg_time = rclcpp::Time(msg->header.stamp);
    // handle old messages
    if (msg_time < _last_message_times["altimeter"]) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Altimeter message received with non-increasing timestamp. Ignoring this message.");
        return;
    }
    _last_message_times["altimeter"] = msg_time;

    _latest_sensor_data.altimeter = msg;
    _altimeter_data_received = true;

    // Only process altimeter measurements after initial home state is set
    if (!_initial_home_state_set) {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *this->get_clock(), 1000,
                             "Altimeter data received but initial home state not set yet. Storing for initialization.");
        return;
    }

    double altitude_offset = msg->vertical_position - _ref_altitude_altimeter;

    Eigen::VectorXd measurement(2);
    measurement(0) = altitude_offset;
    measurement(1) = msg->vertical_velocity;

    Eigen::MatrixXd covariance(2, 2);
    covariance = _R_altimeter;

    auto measurement_ptr = std::make_shared<Measurement>(MeasurementType::ALTIMETER, rclcpp::Time(msg->header.stamp),
                                                         measurement, covariance);
    _filter->addMeasurement(measurement_ptr);
}

void LocalizationCore::odometryGroundTruthCallback(const Odometry::SharedPtr msg) {
    RCLCPP_INFO_STREAM_ONCE(get_logger(), "Odometry ground truth data received");
    _latest_sensor_data.odom = msg;
    _odometry_ground_truth_data_received = true;
}

// utilities
bool LocalizationCore::good_gps(const GpsFix &gps_data) {
    return (gps_data.status.status != GpsStatus::STATUS_NO_FIX && !std::isnan(gps_data.latitude) &&
            !std::isnan(gps_data.longitude) && !std::isnan(gps_data.altitude));
}

double LocalizationCore::computeMagHeading(const MagneticField &mag_data) {
    return std::atan2(mag_data.magnetic_field.y, mag_data.magnetic_field.x);
}

double LocalizationCore::computeMagHeadingCovariance(std::vector<double> std_dev, const MagneticField &mag_data) {
    double mx = mag_data.magnetic_field.x;
    double my = mag_data.magnetic_field.y;
    double denom = mx * mx + my * my;
    if (denom < 1e-6) {
        return 0.0; // avoid division by zero
    }
    double dtheta_dmx = -my / denom;
    double dtheta_dmy = mx / denom;

    double var_x = std_dev[0] * std_dev[0];
    double var_y = std_dev[1] * std_dev[1];

    // assume axes are uncorrelated
    double heading_variance = dtheta_dmx * dtheta_dmx * var_x + dtheta_dmy * dtheta_dmy * var_y;
    return heading_variance;
}

void LocalizationCore::computeOdometryError(const Odometry &odom, const Odometry &ground_truth, Odometry &error) {
    error.header.stamp = odom.header.stamp;
    error.header.frame_id = odom.header.frame_id;
    error.child_frame_id = odom.child_frame_id;

    // position error
    error.pose.pose.position.x = odom.pose.pose.position.x - ground_truth.pose.pose.position.x;
    error.pose.pose.position.y = odom.pose.pose.position.y - ground_truth.pose.pose.position.y;
    error.pose.pose.position.z = odom.pose.pose.position.z - ground_truth.pose.pose.position.z;

    // orientation error (as angle-axis)
    Eigen::Quaterniond q_odom(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                              odom.pose.pose.orientation.z);
    Eigen::Quaterniond q_gt(ground_truth.pose.pose.orientation.w, ground_truth.pose.pose.orientation.x,
                            ground_truth.pose.pose.orientation.y, ground_truth.pose.pose.orientation.z);
    // compute the relative rotation as compute its quaternion
    Eigen::Quaterniond q_err = q_odom * q_gt.conjugate();
    error.pose.pose.orientation.x = q_err.x();
    error.pose.pose.orientation.y = q_err.y();
    error.pose.pose.orientation.z = q_err.z();
    error.pose.pose.orientation.w = q_err.w();

    // velocity error
    error.twist.twist.linear.x = odom.twist.twist.linear.x - ground_truth.twist.twist.linear.x;
    error.twist.twist.linear.y = odom.twist.twist.linear.y - ground_truth.twist.twist.linear.y;
    error.twist.twist.linear.z = odom.twist.twist.linear.z - ground_truth.twist.twist.linear.z;

    error.twist.twist.angular.x = odom.twist.twist.angular.x - ground_truth.twist.twist.angular.x;
    error.twist.twist.angular.y = odom.twist.twist.angular.y - ground_truth.twist.twist.angular.y;
    error.twist.twist.angular.z = odom.twist.twist.angular.z - ground_truth.twist.twist.angular.z;
}

} // namespace localization_kf_core

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(localization_kf_core::LocalizationCore)