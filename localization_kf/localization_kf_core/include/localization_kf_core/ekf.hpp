#ifndef LOCALIZATION_KF_CORE_INCLUDE_LOCALIZATION_KF_CORE_EKF_HPP
#define LOCALIZATION_KF_CORE_INCLUDE_LOCALIZATION_KF_CORE_EKF_HPP

#include <Eigen/Eigen>
#include <cmath>
#include <memory>
#include <mutex>
#include <queue>
#include <vector>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <rclcpp/rclcpp.hpp>

namespace localization_kf_core {

// Constants

/**
 * @brief Measurement types for the EKF
 */
enum class MeasurementType {
    GPS = 0,
    MAGNETOMETER = 1,
    ALTIMETER = 2,
    IMU = 3,  // NEW: IMU as measurement for angular rates and specific force
};

/**
 * @brief Measurement structure for buffering
 */
struct Measurement {
    MeasurementType type;
    rclcpp::Time timestamp;
    Eigen::VectorXd data;
    Eigen::MatrixXd covariance;
    
    Measurement() = default;
    Measurement(MeasurementType t, const rclcpp::Time &time, const Eigen::VectorXd &d, const Eigen::MatrixXd &cov)
        : type(t), timestamp(time), data(d), covariance(cov) {}
        
        // Comparator for priority queue (earlier timestamps have higher priority)
        bool operator<(const Measurement &other) const { return timestamp > other.timestamp; }
    };
    
    using MeasurementPtr = std::shared_ptr<Measurement>;
    using MeasurementQueue = std::priority_queue<MeasurementPtr, std::vector<MeasurementPtr>, std::greater<MeasurementPtr>>;
    
    struct ControlInput {
        rclcpp::Time timestamp;
        Eigen::VectorXd data;
        ControlInput() = default;
    ControlInput(const rclcpp::Time &time, const Eigen::VectorXd &d) : timestamp(time), data(d) {}

    // Comparator for priority queue (earlier timestamps have higher priority)
    bool operator<(const ControlInput &other) const { return timestamp > other.timestamp; }
};
using ControlInputPtr = std::shared_ptr<ControlInput>;
using ControlInputQueue =
    std::priority_queue<ControlInputPtr, std::vector<ControlInputPtr>, std::greater<ControlInputPtr>>;

/**
 * @brief Extended Kalman Filter for uav localization
 * State vector: [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
 * Control inputs: [ax_body, ay_body, az_body, p, q, r]^T (6 inputs from IMU)
 */
class EKF {
    public:
    static constexpr int STATE_SIZE = 12;  // Extended: x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r
    static constexpr int CONTROL_SIZE = 6;
    static constexpr int MAX_CONTROL_QUEUE_SIZE = 100; // ~10 seconds at 100 Hz
    static constexpr double GRAVITY = 9.81; // m/s²
    
    using Odom = nav_msgs::msg::Odometry;
    using AccelWithCovarianceStamped = geometry_msgs::msg::AccelWithCovarianceStamped;

    /**
     * @brief Construct a new EKF object
     * @param process_noise_covariance Process noise covariance matrix (size 12 * 12)
     * @param sensor_timeout Maximum allowed time difference for sensor data
     */
    EKF(const rclcpp::Duration &sensor_timeout, const std::vector<double> &process_noise_cov);
    /**
     * @brief Destroy the EKF object
     *
     */
    ~EKF();

    /**
     * @brief Initialize the EKF with initial state and covariance
     *
     * @param initial_state Initial state vector (size 12)
     * @param initial_covariance Initial covariance matrix (size 12x12)
     */
    void initialize(const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &initial_covariance);

    /**
     * @brief Check if the EKF is initialized
     * @return true if initialized, false otherwise
     */
    bool isInitialized() const;

    /**
     * @brief Reset the EKF to uninitialized state
     *
     */
    void reset();

    /**
     * @brief Predict the next state and covariance based on control input and time delta
     *
     * @param control_input Control input to process
     * @param dt Time delta since last control input
     */
    void predict(const Eigen::VectorXd &control_input, double dt);

    /**
     * @brief Update the state and covariance based on measurement
     *
     * @param h Measurement model
     * @param H Measurement model Jacobian
     * @param innovation Measurement innovation (z - h)
     * @param R Measurement noise covariance
     */
    void update(const Eigen::MatrixXd &H, const Eigen::VectorXd &innovation, const Eigen::MatrixXd &R);

    /**
     * @brief Add a new control input to the queue
     *
     * @param control_input Control input message
     */
    void addControlInput(ControlInputPtr &control_input);

    /**
     * @brief Add a new measurement to the queue
     *
     * @param measurement Measurement message
     */
    void addMeasurement(MeasurementPtr &measurement);

    /**
     * @brief Process control buffer for state prediction
     *  @param time Current time for prediction
     */
    void processControlBuffer(const rclcpp::Time &time);

    /**
     * @brief Process a single control input for state prediction
     * @param control_input Control input to process
     */
    void processControlInput(const ControlInput &control_input);

    /**
     * @brief Generate the state transition function
     * @param state Current state vector
     * @param control_input Current control input vector
     * @param dt Time delta
     * @return Predicted state vector
     */
    Eigen::VectorXd stateFunction(const Eigen::VectorXd &state, const Eigen::VectorXd &control_input, double dt);

    /**
     * @brief Compute the Jacobian of the state transition function
     * @param state Current state vector
     * @param control_input Current control input vector
     * @param dt Time delta
     * @return State transition Jacobian matrix
     */
    Eigen::MatrixXd computeStateTransitionJacobian(const Eigen::VectorXd &state, const Eigen::VectorXd &control_input,
                                                   double dt);

    /**
     * @brief Return the rotation matrix from body to world frame based on current Euler angles
     *
     * @return Rotation matrix (3x3)
     */
    Eigen::MatrixXd rotationMatrixBodyToWorld(Eigen::VectorXd euler_angles) const;

    /**
     * @brief Return the Euler rate transformation matrix based on current Euler angles
     *
     * @return Transformation matrix (3x3)
     */
    Eigen::Matrix3d eulerRateTransformationMatrix(Eigen::VectorXd euler_angles);

    /**
     * @brief Process a new measurement
     * @param time Current time for prediction
     */
    void processMeasurementBuffer(const rclcpp::Time time);

    /**
     * @brief Process a single measurement (robot_localization approach)
     * @param measurement Measurement to process
     */
    void processSingleMeasurement(const Measurement &measurement);

    /**
     * @brief Obtain the measurement model and its Jacobian for a given measurement type
     *
     * @param measurement_type Type of the measurement
     * @return std::pair<Eigen::MatrixXd, Eigen::MatrixXd> Measurement model and its Jacobian
     */

    void getMeasurementModelAndJacobian(const Measurement &measurement, Eigen::VectorXd &h, Eigen::MatrixXd &H);

    /**
     * @brief Get gps measurement model
     * @param state Current state vector
     *
     */
    Eigen::VectorXd gpsMeasurementFunction(const Eigen::VectorXd &state);

    /**
     * @brief Get gps measurement Jacobian
     * @param state Current state vector
     *
     */
    Eigen::MatrixXd gpsMeasurementJacobian(const Eigen::VectorXd &state);

    /**
     * @brief Get magnetometer measurement model
     * @param state Current state vector
     *
     */
    Eigen::VectorXd magnetometerMeasurementFunction(const Eigen::VectorXd &state);

    /**
     * @brief Get magnetometer measurement Jacobian
     * @param state Current state vector
     *
     */
    Eigen::MatrixXd magnetometerMeasurementJacobian(const Eigen::VectorXd &state);

    /**
     * @brief Get altimeter measurement model
     * @param state Current state vector
     *
     */
    Eigen::VectorXd altimeterMeasurementFunction(const Eigen::VectorXd &state);

    /**
     * @brief Get altimeter measurement Jacobian
     * @param state Current state vector
     */
    Eigen::MatrixXd altimeterMeasurementJacobian(const Eigen::VectorXd &state);

    /**
     * @brief Get current odometry state
     * @param odometry_msg Output odometry message
     *  @return bool True if odometry is valid, false otherwise
     */
    bool getOdometry(Odom &odometry_msg) const;

    /**
     * @brief Get current acceleration estimates
     * @param acceleration_msg Output acceleration message
     * @return bool True if acceleration is valid, false otherwise
     */
    bool getAcceleration(AccelWithCovarianceStamped &acceleration_msg) const;

    /**
     * @brief Normalize angles in the state vector to [-pi, pi]
     * @param state State vector to normalize
     * @brief Normalize state angles to [-pi, pi]
     * @param state State vector to normalize
     */
    void normalizeStateAngles(Eigen::VectorXd &state);

    /**
     * @brief Wrap angle to [-pi, pi]
     * @param angle Angle to wrap
     * @return Wrapped angle
     */
    double wrapAngle(double angle);

    /**
     * @brief Convert Euler angles to quaternion
     * @param euler_angles Euler angles (roll, pitch, yaw)
     * @return Quaternion
     */
    static Eigen::Quaterniond euler_to_quaternion(const Eigen::Vector3d &euler);

  private:
    Eigen::VectorXd _state;      // State vector
    Eigen::MatrixXd _covariance; // State covariance matrix
    std::mutex _state_mutex;     // Mutex for thread safety

    // Queues for measurements and control inputs
    MeasurementQueue _measurement_queue;
    ControlInputQueue _control_input_queue;

    // mutexes for queues
    mutable std::mutex _measurement_mutex;   // Mutex for measurement queue
    mutable std::mutex _control_input_mutex; // Mutex for control input queue

    // Last control input
    ControlInputPtr _last_control_input{nullptr};
    rclcpp::Time _last_control_time;
    // Last update time
    rclcpp::Time _last_measurement_time;
    rclcpp::Duration _sensor_timeout; // Maximum allowed time difference for sensor data
    
    // Per-sensor last update times for rate limiting (robot_localization technique)
    std::map<MeasurementType, rclcpp::Time> _last_sensor_update_times;

    // process noise covariance
    Eigen::MatrixXd _Q; // Process noise covariance matrix

    // Robot_localization-style filtering parameters
    // Flags
    bool _is_initialized{false};
};
} // namespace localization_kf_core

#endif // LOCALIZATION_KF_CORE_INCLUDE_LOCALIZATION_KF_CORE_EKF_HPP