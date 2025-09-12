
#ifndef LOCALIZATION_KF_RVIZ_PLUGIN_NAVIGATION_PANEL_HPP
#define LOCALIZATION_KF_RVIZ_PLUGIN_NAVIGATION_PANEL_HPP

#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QComboBox>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "localization_kf_msgs/action/start_navigation.hpp"
#include <QBasicTimer>

namespace localization_kf_rviz_plugin
{
    class NavigationPanel : public rviz_common::Panel
    {
        Q_OBJECT

    public:
        using StartNavigationAction = localization_kf_msgs::action::StartNavigation;
        using StartNavigationActionGoal = localization_kf_msgs::action::StartNavigation::Goal;
        using GoalHandleStartNavigation = rclcpp_action::ClientGoalHandle<StartNavigationAction>;

        /**
         * @brief Construct a new Navigation Panel object
         *
         * @param parent
         */
        explicit NavigationPanel(QWidget *parent = 0);

        /**
         * @brief Destroy the Navigation Panel object
         *
         */
        ~NavigationPanel() override;

        /**
         * @brief Initialize the Navigation Panel
         *
         */
        void onInitialize() override;

    private Q_SLOTS:
        /**
         * @brief Start the navigation process
         *
         */
        void onStartNavigationClicked();

        /**
         * @brief  start navigation feedback callback
         *
         * @param goal_handle
         * @param feedback
         */
        void startNavigationFeedbackCallback(
            const GoalHandleStartNavigation::SharedPtr &goal_handle,
            const std::shared_ptr<const StartNavigationAction::Feedback> feedback);

        /**
         * @brief  start navigation result callback
         *
         * @param result
         */
        void startNavigationResultCallback(
            const GoalHandleStartNavigation::WrappedResult &result);

        /**
         * @brief  start navigation goal response callback
         *
         * @param goal_handle
         */
        void startNavigationGoalResponseCallback(
            const GoalHandleStartNavigation::SharedPtr &goal_handle);

    protected:
        /**
         * @brief Get the Selected Trajectory object
         *
         * @return int
         */
        int getSelectedTrajectory() const;

        QPushButton *_start_navigation_button;
        QPushButton *_return_home_button;
        QComboBox *_trajectory_selector;
        QLabel *_status_label;

        // Action clients for navigation and home return
        rclcpp_action::Client<StartNavigationAction>::SharedPtr _start_navigation_action_client;

        // to check completion of action
        QBasicTimer timer_;

        // create ros2 node
        rclcpp::Node::SharedPtr _client_node;
    };
}

#endif // LOCALIZATION_KF_RVIZ_PLUGIN_NAVIGATION_PANEL_HPP
