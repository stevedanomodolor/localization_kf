#include <localization_kf_core/ekf.hpp>

namespace localization_kf_core {

EKF::EKF(const rclcpp::Duration &sensor_timeout, const std::vector<double> &process_noise_cov)
    : _sensor_timeout(sensor_timeout), _is_initialized(false) {
    if (process_noise_cov.size() != (STATE_SIZE * STATE_SIZE)) {
        throw std::invalid_argument("Process noise covariance vector has incorrect size");
    }

    // fill up _Q matrix
    _Q = Eigen::MatrixXd(STATE_SIZE, STATE_SIZE);
    for (int i = 0; i < STATE_SIZE; ++i) {
        for (int j = 0; j < STATE_SIZE; ++j) {
            _Q(i, j) = process_noise_cov[i * STATE_SIZE + j];
        }
    }

    _last_control_time = rclcpp::Time(0, 0);
    _last_measurement_time = rclcpp::Time(0, 0);
}

EKF::~EKF() {}

void EKF::initialize(const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &initial_covariance) {

    std::lock_guard<std::mutex> lock(_state_mutex);
    if (initial_state.size() != STATE_SIZE) {
        throw std::invalid_argument("Initial state vector has incorrect size");
    }
    if (initial_covariance.rows() != STATE_SIZE || initial_covariance.cols() != STATE_SIZE) {
        throw std::invalid_argument("Initial covariance matrix has incorrect size");
    }
    // Initialize state and covariance
    _state = initial_state;
    _covariance = initial_covariance;
    // print 
    _last_control_input = nullptr;

    _is_initialized = true;
    
    RCLCPP_INFO(rclcpp::get_logger("EKF"), "EKF initialized with state norm: %.6f, covariance det: %.6e", 
               _state.norm(), _covariance.determinant());
}

bool EKF::isInitialized() const { return _is_initialized; }

void EKF::reset() {
    std::lock_guard<std::mutex> lock(_state_mutex);
    _is_initialized = false;
    _state = Eigen::VectorXd::Zero(STATE_SIZE);
    _covariance = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) * 1e6; // large initial uncertainty
    _last_control_input = nullptr;
}

void EKF::addControlInput(ControlInputPtr &control_input) {
    std::lock_guard<std::mutex> lock(_control_input_mutex);
    if (_control_input_queue.size() >= MAX_CONTROL_QUEUE_SIZE) {
        // RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000, "Control input queue full, dropping oldest
        // input");
        _control_input_queue.pop();
    }
    _control_input_queue.push(control_input);
}

void EKF::addMeasurement(MeasurementPtr &measurement) {
    std::lock_guard<std::mutex> lock(_measurement_mutex);
    _measurement_queue.push(measurement);
}

void EKF::processControlBuffer(const rclcpp::Time &time) {
    // RCLCPP_INFO(rclcpp::get_logger("EKF"), "Processing control buffer at time: %f", time.seconds());
    std::lock_guard<std::mutex> lock(_control_input_mutex);
    if (!_is_initialized) {
        RCLCPP_WARN(rclcpp::get_logger("EKF"), "EKF not initialized, cannot process control inputs");
        return;
    }

    bool control_input_processed = false;
    rclcpp::Time last_processed_time = _last_control_time;

    // Process all control inputs up to the given time
    while (!_control_input_queue.empty()) {
        ControlInputPtr control_input = _control_input_queue.top();
        if (control_input->timestamp <= time) {
            // Process the control input
            _control_input_queue.pop();
            processControlInput(*control_input);
            control_input_processed = true;
            last_processed_time = control_input->timestamp;
        } else {
            break;
        }
    }

    // Trim control input queue to max size
    while (_control_input_queue.size() > MAX_CONTROL_QUEUE_SIZE) {
        _control_input_queue.pop();
    }
}

void EKF::processControlInput(const ControlInput &control_input) {

    double dt = control_input.timestamp.seconds() - _last_control_time.seconds();
    if (_last_control_time.seconds() != 0 || _last_control_time.nanoseconds() != 0) {
        dt = (control_input.timestamp - _last_control_time).seconds();

    
        if (dt <= 0.0) {
            return;
        }
        if (dt > _sensor_timeout.seconds()) {
            return;
        }
    } else {
        // First control input, just set last control time
        _last_control_time = control_input.timestamp;
        RCLCPP_DEBUG(rclcpp::get_logger("EKF"), "First control input received, setting last control time");
        return;
    }


    // Extract control inputs
    if (control_input.data.size() != CONTROL_SIZE) {
        RCLCPP_WARN(rclcpp::get_logger("EKF"), "Control input vector has incorrect size");
        return;
    }

    // predict state and covariance
    predict(control_input.data, dt);
    // # TODO maybe move outsie not to use make share again
    _last_control_input = std::make_shared<ControlInput>(control_input);
    _last_control_time = control_input.timestamp;
}

void EKF::predict(const Eigen::VectorXd &control_input, double dt) {
    std::lock_guard<std::mutex> lock(_state_mutex);

    // State transition function
    Eigen::VectorXd predicted_state = stateFunction(_state, control_input, dt);

    // Compute Jacobian of the state transition function
    Eigen::MatrixXd F = computeStateTransitionJacobian(_state, control_input, dt);

    // Process noise covariance
    Eigen::MatrixXd predicted_covariance = F * _covariance * F.transpose() + _Q * dt;

    // Update state and covariance
    _state = predicted_state;
    _covariance = predicted_covariance;

    // normalize angles
    normalizeStateAngles(_state);
}


void EKF::update(const Eigen::MatrixXd &H, const Eigen::VectorXd &innovation, const Eigen::MatrixXd &R) {
    std::lock_guard<std::mutex> lock(_state_mutex);

    Eigen::MatrixXd R_stable = R;

    // Handle negative covariances and very small values (robot_localization technique)
    for (int i = 0; i < R_stable.rows(); ++i) {
        if (R_stable(i, i) < 0) {
            RCLCPP_WARN(rclcpp::get_logger("EKF"),
                        "WARNING: Negative covariance for measurement index %d (value: %.6f). Using absolute value.", i,
                        R_stable(i, i));
            R_stable(i, i) = std::fabs(R_stable(i, i));
        }

        // Add small noise to maintain filter stability for near-zero covariances
        if (R_stable(i, i) < 1e-9) {
            RCLCPP_WARN(
                rclcpp::get_logger("EKF"),
                "WARNING: Measurement had very small error covariance for index %d. Adding noise for stability.", i);
            R_stable(i, i) = 1e-9;
        }
    }

    // Create wrapped innovation for angle states
    Eigen::VectorXd innovation_wrapped = innovation;

    // Wrap angles in the innovation (indices 6, 7, 8 are roll, pitch, yaw)
    if (innovation_wrapped.size() >= 3 && H.rows() >= 3) {
        for (int i = 0; i < H.rows(); ++i) {
            // Check if this measurement row corresponds to an angle state
            if (H(i, 6) != 0) { // roll
                innovation_wrapped(i) = wrapAngle(innovation_wrapped(i));
            } else if (H(i, 7) != 0) { // pitch
                innovation_wrapped(i) = wrapAngle(innovation_wrapped(i));
            } else if (H(i, 8) != 0) { // yaw
                innovation_wrapped(i) = wrapAngle(innovation_wrapped(i));
            }
        }
    }

    // Eigen::MatrixXd S = H * _covariance * H.transpose() + R_stable; // Innovation covariance
    // Eigen::MatrixXd K = _covariance * H.transpose() * S.inverse();               // Kalman gain
    // _state = _state + K * innovation_wrapped;                                    // Updated state
    // Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    // _covariance = (I - K * H) * _covariance;                                     // Updated covariance

    // use joseph form to improve numerical stability
    Eigen::MatrixXd S = H * _covariance * H.transpose() + R_stable; // Innovation covariance
    Eigen::MatrixXd K = _covariance * H.transpose() * S.inverse();               // Kalman gain
    _state = _state + K * innovation_wrapped;                                    // Updated state
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    _covariance = (I - K * H) * _covariance * (I - K * H).transpose() + K * R_stable * K.transpose(); // Joseph form
    normalizeStateAngles(_state);
}

void EKF::processMeasurementBuffer(const rclcpp::Time time) {
    std::lock_guard<std::mutex> lock(_measurement_mutex);
    if (!_is_initialized) {
        RCLCPP_WARN(rclcpp::get_logger("EKF"), "EKF not initialized, cannot process measurements");
        return;
    }

    // Process all measurements up to the given time
    while (!_measurement_queue.empty()) {

        MeasurementPtr measurement = _measurement_queue.top();
        if (measurement->timestamp <= time) {
            // Process the measurement
            _measurement_queue.pop();

            // set _last_measurement_time for the first measurement
            if (_last_measurement_time.seconds() == 0 && _last_measurement_time.nanoseconds() == 0) {
                _last_measurement_time = measurement->timestamp;
                continue;
            }

            processSingleMeasurement(*measurement);

            // Update last measurement time
            _last_measurement_time = measurement->timestamp;

        } else {
            break;
        }
    }
}

void EKF::processSingleMeasurement(const Measurement &measurement) {

    Eigen::VectorXd h;                          // Expected measurement
    Eigen::MatrixXd H;                          // Measurement Jacobian
    Eigen::MatrixXd R = measurement.covariance; // Measurement noise covariance

    try {
        getMeasurementModelAndJacobian(measurement, h, H);
    } catch (const std::exception &e) {
        RCLCPP_WARN(rclcpp::get_logger("EKF"), "Failed to obtain measurement model: %s", e.what());
        return;
    }
    

    // Innovation
    Eigen::VectorXd innovation = measurement.data - h;

    // Correct state and covariance
    update(H, innovation, R);
    
  }

void EKF::getMeasurementModelAndJacobian(const Measurement &measurement, Eigen::VectorXd &h, Eigen::MatrixXd &H) {
    std::lock_guard<std::mutex> lock(_state_mutex);

    if (measurement.type == MeasurementType::GPS) {
        // GPS update (6dof: position + velocity)
        h = gpsMeasurementFunction(_state);
        H = gpsMeasurementJacobian(_state);
    } else if (measurement.type == MeasurementType::MAGNETOMETER) {
        // Magnetometer update (1dof: yaw)
        h = magnetometerMeasurementFunction(_state);
        H = magnetometerMeasurementJacobian(_state);
    } else if (measurement.type == MeasurementType::ALTIMETER) {
        // Altimeter update (2dof: altitude + vertical velocity)
        h = altimeterMeasurementFunction(_state);
        H = altimeterMeasurementJacobian(_state);
    } else {
        throw std::invalid_argument("Unknown measurement type");
    }
}

Eigen::VectorXd EKF::gpsMeasurementFunction(const Eigen::VectorXd &state) {
    Eigen::VectorXd h(6);
    h.head<3>() = state.head<3>();     // Position
    h.tail<3>() = state.segment<3>(3); // Velocity
    return h;
}

Eigen::MatrixXd EKF::gpsMeasurementJacobian(const Eigen::VectorXd &state) {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, STATE_SIZE);
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(); // Position
    H.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity(); // Velocity
    return H;
}

Eigen::VectorXd EKF::magnetometerMeasurementFunction(const Eigen::VectorXd &state) {
    Eigen::VectorXd h(1);
    h(0) = state(8); // Yaw (psi)
    return h;
}

Eigen::MatrixXd EKF::magnetometerMeasurementJacobian(const Eigen::VectorXd &state) {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, STATE_SIZE);
    H(0, 8) = 1.0; // Yaw (psi)
    return H;
}

Eigen::VectorXd EKF::altimeterMeasurementFunction(const Eigen::VectorXd &state) {
    Eigen::VectorXd h(2);
    h(0) = state(2); // Altitude (z)
    h(1) = state(5); // Vertical velocity (vz)
    return h;
}

Eigen::MatrixXd EKF::altimeterMeasurementJacobian(const Eigen::VectorXd &state) {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, STATE_SIZE);
    H(0, 2) = 1.0; // Altitude (z)
    H(1, 5) = 1.0; // Vertical velocity (vz)
    return H;
}


Eigen::MatrixXd EKF::rotationMatrixBodyToWorld(Eigen::VectorXd euler_angles) const {
    double phi = euler_angles(0);   // roll
    double theta = euler_angles(1); // pitch
    double psi = euler_angles(2);   // yaw

    double cos_phi = cos(phi), sin_phi = sin(phi);
    double cos_theta = cos(theta), sin_theta = sin(theta);
    double cos_psi = cos(psi), sin_psi = sin(psi);

    Eigen::Matrix3d R;
    R << cos_theta * cos_psi, -cos_phi * sin_psi + sin_phi * sin_theta * cos_psi,
        sin_phi * sin_psi + cos_phi * sin_theta * cos_psi, cos_theta * sin_psi,
        cos_phi * cos_psi + sin_phi * sin_theta * sin_psi, -sin_phi * cos_psi + cos_phi * sin_theta * sin_psi,
        -sin_theta, sin_phi * cos_theta, cos_phi * cos_theta;
    return R;
}

Eigen::Matrix3d EKF::eulerRateTransformationMatrix(Eigen::VectorXd euler_angles) {
    double phi = euler_angles(0);   // roll
    double theta = euler_angles(1); // pitch

    double cos_theta = cos(theta);
    double sin_phi = sin(phi);
    double cos_phi = cos(phi);
    double tan_theta = tan(theta);

    Eigen::Matrix3d T;
    T << 1, sin_phi * tan_theta, cos_phi * tan_theta, 0, cos_phi, -sin_phi, 0, sin_phi / cos_theta, cos_phi / cos_theta;
    return T;
}

Eigen::VectorXd EKF::stateFunction(const Eigen::VectorXd &state, const Eigen::VectorXd &control_input, double dt) {
    // Extract state components (15 states: x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r)
    Eigen::Vector3d position = state.segment<3>(0);      // x, y, z
    Eigen::Vector3d velocity = state.segment<3>(3);      // vx, vy, vz
    Eigen::Vector3d euler_angles = state.segment<3>(6);  // roll, pitch, yaw
    Eigen::Vector3d angular_rates = state.segment<3>(9); // p, q, r

   
    Eigen::Matrix3d R = rotationMatrixBodyToWorld(euler_angles);

    // Euler rate transformation matrix
    Eigen::Matrix3d T = eulerRateTransformationMatrix(euler_angles);
    Eigen::Vector3d euler_rate = T * angular_rates;

    Eigen::VectorXd next_state(STATE_SIZE);

    // Control inputs: ax, ay, az (body frame)
    Eigen::Vector3d body_acceleration = control_input.segment<3>(0);
    Eigen::Vector3d acceleration = R * body_acceleration; // Convert to world frame

    // Constant acceleration model
    // Position update: p_k+1 = p_k + v_k*dt
    next_state.segment<3>(0) = position + velocity * dt;

    // Velocity update: v_k+1 = v_k + a_k*dt
    next_state.segment<3>(3) = velocity + acceleration * dt * 0.05;  // 5% influence

    // Euler angles update
    next_state.segment<3>(6) = euler_angles + euler_rate * dt;

    // Angular rates remain approximately constant (no angular acceleration model)
    next_state.segment<3>(9) = angular_rates;

    return next_state;
}

Eigen::MatrixXd EKF::computeStateTransitionJacobian(const Eigen::VectorXd &state, const Eigen::VectorXd &control_input,
                                                    double dt) {
    // Initialize as identity matrix
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);

    // Extract state components (12 states: x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r)
    Eigen::Vector3d euler_angles = state.segment<3>(6);  // roll, pitch, yaw
    Eigen::Vector3d angular_rates = state.segment<3>(9); // p, q, r

    double phi = euler_angles(0);    // roll
    double theta = euler_angles(1);  // pitch
    double psi = euler_angles(2);    // yaw

    double p_in = angular_rates(0);  // roll rate
    double q_in = angular_rates(1);  // pitch rate
    double r_in = angular_rates(2);  // yaw rate

    // Extract control inputs (body frame accelerations)
    double abx = control_input(0);  // acceleration in body x
    double aby = control_input(1);  // acceleration in body y
    double abz = control_input(2);  // acceleration in body z

    // Precompute trigonometric functions
    double sin_phi = sin(phi), cos_phi = cos(phi);
    double sin_theta = sin(theta), cos_theta = cos(theta);
    double sin_psi = sin(psi), cos_psi = cos(psi);
    double tan_theta = tan(theta);

    // Fill the Jacobian matrix based on your provided values:

    // Position derivatives w.r.t. velocity (rows 0-2, cols 3-5)
    F(0, 3) = dt;  // dx/dvx
    F(1, 4) = dt;  // dy/dvy  
    F(2, 5) = dt;  // dz/dvz

    // Velocity derivatives w.r.t. orientation (rows 3-5, cols 6-8)
    // Row 3 (vx derivatives)
    F(3, 6) = dt * (aby * (sin_phi * sin_psi + sin_theta * cos_phi * cos_psi) + 
                    abz * (-sin_phi * sin_theta * cos_psi + sin_psi * cos_phi));
    F(3, 7) = dt * (-abx * sin_theta * cos_psi + 
                    aby * sin_phi * cos_psi * cos_theta + 
                    abz * cos_phi * cos_psi * cos_theta);
    F(3, 8) = dt * (-abx * sin_psi * cos_theta + 
                    aby * (-sin_phi * sin_psi * sin_theta - cos_phi * cos_psi) + 
                    abz * (sin_phi * cos_psi - sin_psi * sin_theta * cos_phi));

    // Row 4 (vy derivatives)
    F(4, 6) = dt * (aby * (-sin_phi * cos_psi + sin_psi * sin_theta * cos_phi) + 
                    abz * (-sin_phi * sin_psi * sin_theta - cos_phi * cos_psi));
    F(4, 7) = dt * (-abx * sin_psi * sin_theta + 
                    aby * sin_phi * sin_psi * cos_theta + 
                    abz * sin_psi * cos_phi * cos_theta);
    F(4, 8) = dt * (abx * cos_psi * cos_theta + 
                    aby * (sin_phi * sin_theta * cos_psi - sin_psi * cos_phi) + 
                    abz * (sin_phi * sin_psi + sin_theta * cos_phi * cos_psi));

    // Row 5 (vz derivatives)
    F(5, 6) = dt * (aby * cos_phi * cos_theta - abz * sin_phi * cos_theta);
    F(5, 7) = dt * (-abx * cos_theta - aby * sin_phi * sin_theta - abz * sin_theta * cos_phi);
    // F(5, 8) = 0 (already set by identity)

    // Euler angle derivatives w.r.t. euler angles and angular rates (rows 6-8)
    // Row 6 (roll derivatives)
    F(6, 6) = 1 + dt * (q_in * cos_phi * tan_theta - r_in * sin_phi * tan_theta);
    F(6, 7) = dt * (q_in * (tan_theta * tan_theta + 1) * sin_phi + 
                    r_in * (tan_theta * tan_theta + 1) * cos_phi);
    // F(6, 8) = 0 (already set by identity)
    // F(6, 9) = 0, F(6, 10) = 0, F(6, 11) = 0 (will be set when we add control derivatives)

    // Row 7 (pitch derivatives)
    F(7, 6) = dt * (-q_in * sin_phi - r_in * cos_phi);
    // F(7, 7) = 1 (already set by identity)
    // F(7, 8) = 0, F(7, 9) = 0, F(7, 10) = 0, F(7, 11) = 0

    // Row 8 (yaw derivatives)
    F(8, 6) = dt * (q_in * cos_phi / cos_theta - r_in * sin_phi / cos_theta);
    F(8, 7) = dt * (q_in * sin_phi * sin_theta / (cos_theta * cos_theta) + 
                    r_in * sin_theta * cos_phi / (cos_theta * cos_theta));
    // F(8, 8) = 1 (already set by identity)
    // F(8, 9) = 0, F(8, 10) = 0, F(8, 11) = 0

    // Angular rates are directly updated from control input, so rows 9-11 remain identity
    // F(9, 9) = 1, F(10, 10) = 1, F(11, 11) = 1 (already set by identity)

    return F;
}

bool EKF::getOdometry(Odom &odometry_msg) const {
    if (!_is_initialized)
        return false;

    std::lock_guard<std::mutex> lock(const_cast<std::mutex &>(_state_mutex));

    odometry_msg.header.stamp = rclcpp::Clock().now();
    odometry_msg.header.frame_id = "map";
    odometry_msg.child_frame_id = "base_link";

    // Position
    odometry_msg.pose.pose.position.x = _state(0);
    odometry_msg.pose.pose.position.y = _state(1);
    odometry_msg.pose.pose.position.z = _state(2);

    // Orientation (convert Euler to quaternion)
    Eigen::Vector3d euler = _state.segment<3>(6);
    Eigen::Quaterniond q = euler_to_quaternion(euler);
    odometry_msg.pose.pose.orientation.x = q.x();
    odometry_msg.pose.pose.orientation.y = q.y();
    odometry_msg.pose.pose.orientation.z = q.z();
    odometry_msg.pose.pose.orientation.w = q.w();

    // Velocity - transform from world frame to body frame
    Eigen::Vector3d world_velocity = _state.segment<3>(3);
    Eigen::Matrix3d R_bg = rotationMatrixBodyToWorld(euler);
    Eigen::Vector3d body_velocity = R_bg.transpose() * world_velocity;

    odometry_msg.twist.twist.linear.x = body_velocity(0);
    odometry_msg.twist.twist.linear.y = body_velocity(1);
    odometry_msg.twist.twist.linear.z = body_velocity(2);

    // Angular velocity (already in body frame)
    odometry_msg.twist.twist.angular.x = _state(9);
    odometry_msg.twist.twist.angular.y = _state(10);
    odometry_msg.twist.twist.angular.z = _state(11);

    // Covariance (6x6 for pose, 6x6 for twist)
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            odometry_msg.pose.covariance[i * 6 + j] = _covariance(i, j);
            odometry_msg.twist.covariance[i * 6 + j] = _covariance(i + 6, j + 6);
        }
    }

    return true;
}

bool EKF::getAcceleration(AccelWithCovarianceStamped &acceleration_msg) const {
    if (!_is_initialized)
        return false;

    if (!_last_control_input)
        return false;

    acceleration_msg.header.stamp = rclcpp::Clock().now();
    acceleration_msg.header.frame_id = "base_link";

    // Return last control input (IMU acceleration in body frame)
    acceleration_msg.accel.accel.linear.x = _last_control_input->data(0);
    acceleration_msg.accel.accel.linear.y = _last_control_input->data(1);
    acceleration_msg.accel.accel.linear.z = _last_control_input->data(2);

    acceleration_msg.accel.accel.angular.x = _last_control_input->data(3);
    acceleration_msg.accel.accel.angular.y = _last_control_input->data(4);
    acceleration_msg.accel.accel.angular.z = _last_control_input->data(5);

    return true;
}

void EKF::normalizeStateAngles(Eigen::VectorXd &state) {
    // Normalize Euler angles to [-pi, pi] (same indices for 18-state system)
    state(6) = wrapAngle(state(6)); // phi
    state(7) = wrapAngle(state(7)); // theta
    state(8) = wrapAngle(state(8)); // psi
}

double EKF::wrapAngle(double angle) {
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

Eigen::Quaterniond EKF::euler_to_quaternion(const Eigen::Vector3d &euler) {
    double phi = euler(0), theta = euler(1), psi = euler(2);

    Eigen::AngleAxisd roll_angle(phi, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(theta, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(psi, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yaw_angle * pitch_angle * roll_angle;
    return q;
}

} // namespace localization_kf_core
