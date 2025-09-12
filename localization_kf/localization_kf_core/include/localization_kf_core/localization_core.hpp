#ifndef LOCALIZATION_KF_CORE_INCLUDE_LOCALIZATION_KF_CORE_LOCALIZATION_CORE_HPP
#define LOCALIZATION_KF_CORE_INCLUDE_LOCALIZATION_KF_CORE_LOCALIZATION_CORE_HPP

#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "localization_kf_core/ekf.hpp"
#include "nav2_util/lifecycle_node.hpp"

#include "GeographicLib/Geocentric.hpp"
#include "GeographicLib/LocalCartesian.hpp"

#include <geometry_msgs/msg/accel_with_covariance.hpp>
#include <gps_msgs/msg/gps_fix.hpp>
#include <gps_msgs/msg/gps_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ros_gz_interfaces/msg/altimeter.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

namespace localization_kf_core {

class LocalizationCore : public nav2_util::LifecycleNode {
  public:
    static constexpr float GRAVITY = 9.81f;
    using GpsFix = gps_msgs::msg::GPSFix;
    using Imu = sensor_msgs::msg::Imu;
    using MagneticField = sensor_msgs::msg::MagneticField;
    using Altimeter = ros_gz_interfaces::msg::Altimeter;
    using Odometry = nav_msgs::msg::Odometry;
    using AccelWithCovarianceStamped = geometry_msgs::msg::AccelWithCovarianceStamped;
    using GpsStatus = gps_msgs::msg::GPSStatus;

    /**
     * @brief Construct a new Localization Core object
     */
    explicit LocalizationCore(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /**
     * @brief Destroy the Localization Core object
     */
    ~LocalizationCore() = default;

  protected:
    /**
     * @brief Configure member variables and initialize EKF
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

    // timer callbacks
    /**
     * @brief Timer callback for processing sensor data
     * @return void
     */
    void timerCallback();

    // subscribers callbacks
    /**
     * @brief Callback for GPS fix messages
     *
     * @param msg GPS fix message
     */
    void gpsFixCallback(const GpsFix::SharedPtr msg);
    /**
     * @brief Callback for IMU messages
     *
     * @param msg IMU message
     */
    void imuCallback(const Imu::SharedPtr msg);
    /**
     * @brief Callback for Magnetic Field messages
     *
     * @param msg Magnetic Field message
     */
    void magneticFieldCallback(const MagneticField::SharedPtr msg);
    /**
     * @brief Callback for Altimeter messages
     *
     * @param msg Altimeter message
     */
    void altimeterCallback(const Altimeter::SharedPtr msg);
    /**
     * @brief Callback for Odometry messages
     *
     * @param msg Odometry message
     */
    void odometryGroundTruthCallback(const Odometry::SharedPtr msg);

    // Functions
    /**
     * @brief Set the initial home state
     * @return bool True if initial home state is set, false otherwise
     */
    bool is_initial_position_set();

    /**
     * @brief Check if the GPS data is valid
     * @param gps_data Pointer to the GPS data
     * @return true if the GPS data is valid, false otherwise
     */
    bool good_gps(const GpsFix &gps_data);

    /**
     * @brief Compute mag heading from magnetic field data
     * @param mag_data Pointer to the magnetic field data
     * @return double Heading in radians
     */
    double computeMagHeading(const MagneticField &mag_data);

    /**
     * @brief Compute mag heading covariance from magnetic field data
     * @param std_dev Standard deviation of magnetic field measurement noise( all have same std dev)
     * @param mag_data Pointer to the magnetic field data
     * @return double Covariance in radians^2
     */
    double computeMagHeadingCovariance(std::vector<double> std_dev, const MagneticField &mag_data);

    /**
     * @brief Compute odometry error between two odometry messages
     * @param odom Current odometry message
     * @param ground_truth Ground truth odometry message
     * @param error Output odometry error message
     */
    void computeOdometryError(const Odometry &odom, const Odometry &ground_truth, Odometry &error);

  private:
    // timer
    rclcpp::TimerBase::SharedPtr _timer;

    struct SensorData {
        GpsFix::SharedPtr gps;
        Imu::SharedPtr imu;
        MagneticField::SharedPtr mag;
        Altimeter::SharedPtr altimeter;
        Odometry::SharedPtr odom;
    } _latest_sensor_data;

    // Publishers
    rclcpp_lifecycle::LifecyclePublisher<Odometry>::SharedPtr _odometry_estimated_pub;
    rclcpp_lifecycle::LifecyclePublisher<AccelWithCovarianceStamped>::SharedPtr _acceleration_estimated_pub;
    rclcpp_lifecycle::LifecyclePublisher<Odometry>::SharedPtr _odometry_estimate_error_pub;

    // subscribers
    rclcpp::Subscription<GpsFix>::SharedPtr _gps_fix_sub;
    rclcpp::Subscription<Imu>::SharedPtr _imu_sub;
    rclcpp::Subscription<MagneticField>::SharedPtr _magnetic_field_sub;
    rclcpp::Subscription<Altimeter>::SharedPtr _altimeter_sub;
    rclcpp::Subscription<Odometry>::SharedPtr _odometry_ground_truth_sub;

    // Variables
    double _frequency{0.0};
    std::shared_ptr<EKF> _filter;
    // Geographic coordinate transformation
    GeographicLib::Geocentric _earth;
    GeographicLib::LocalCartesian _ref_local_cartesian;

    double _ref_altitude_altimeter{0.0};
    double _ref_gps_altitude{0.0};
    double _base_offset{0.0};
    rclcpp::Duration _sensor_timeout;

    // std dev
    Eigen::MatrixXd _R_gps;
    std::vector<double> _magnetic_field_stddev;
    Eigen::MatrixXd _R_altimeter;

    // Message timing tracking
    std::map<std::string, rclcpp::Time> _last_message_times;
    rclcpp::Time _last_control_time;

    // Status flags
    bool _gps_data_received;
    bool _imu_data_received;
    bool _magnetic_field_data_received;
    bool _altimeter_data_received;
    bool _odometry_ground_truth_data_received;
    bool _initial_home_state_set{false};
    
    // Control input configuration
    bool _use_imu_as_control{true};      // Use IMU as control input (prediction)
    bool _use_imu_as_measurement{true};  // Use IMU as measurement (correction)
};

} // namespace localization_kf_core

#endif // LOCALIZATION_KF_CORE_INCLUDE_LOCALIZATION_KF_CORE_LOCALIZATION_CORE_HPP