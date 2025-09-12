#include <localization_kf_rviz_plugin/navigation_panel.hpp>

namespace localization_kf_rviz_plugin
{
  NavigationPanel::NavigationPanel(QWidget *parent) : Panel(parent)
  {

    const auto layout = new QVBoxLayout(this);
    _start_navigation_button = new QPushButton("Start Navigation", this);

    // three trajectories: square, circle, triangle
    _trajectory_selector = new QComboBox(this);
    _trajectory_selector->addItem("Square");
    _trajectory_selector->addItem("Circle");
    _trajectory_selector->addItem("Figure8");

    _status_label = new QLabel("Status: Idle", this);

    layout->addWidget(_start_navigation_button);
    layout->addWidget(_trajectory_selector);
    layout->addWidget(_status_label);

    // setLayout(layout);

    connect(_start_navigation_button, &QPushButton::clicked, this, &NavigationPanel::onStartNavigationClicked);
  }

  NavigationPanel::~NavigationPanel()
  {
    // Clean up resources if necessary
  }

  void NavigationPanel::onInitialize()
  {
    auto node_ptr = getDisplayContext()->getRosNodeAbstraction().lock();

    _client_node = node_ptr->get_raw_node();
    // Initialize ROS node and action clients
    _start_navigation_action_client = rclcpp_action::create_client<StartNavigationAction>(_client_node, "start_navigation");
  }

  void NavigationPanel::onStartNavigationClicked()
  {
    // get current selected trajectory
    int selected_trajectory = getSelectedTrajectory();

    // send start navigation request
    auto goal_msg = StartNavigationAction::Goal();
    goal_msg.trajectory = selected_trajectory;

    _status_label->setText("Status: Sending a start navigation request...");

    bool is_action_ready = _start_navigation_action_client->wait_for_action_server(std::chrono::seconds(5));
    if (!is_action_ready)
    {
      RCLCPP_ERROR(_client_node->get_logger(), "Start navigation action server not available");
      return;
    }

    auto send_goal_options = rclcpp_action::Client<StartNavigationAction>::SendGoalOptions();

    // goal response callback
    send_goal_options.goal_response_callback = std::bind(&NavigationPanel::startNavigationGoalResponseCallback, this, std::placeholders::_1);

    // feedback callback
    send_goal_options.feedback_callback = std::bind(&NavigationPanel::startNavigationFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);

    // navigation result callback
    send_goal_options.result_callback = std::bind(&NavigationPanel::startNavigationResultCallback, this, std::placeholders::_1);

    _start_navigation_action_client->async_send_goal(goal_msg, send_goal_options);
  }

  int NavigationPanel::getSelectedTrajectory() const
  {
    QString selected_text = _trajectory_selector->currentText();
    if (selected_text == "Square")
    {
      return StartNavigationActionGoal::SQUARE;
    }
    else if (selected_text == "Circle")
    {
      return StartNavigationActionGoal::CIRCLE;
    }
    else if (selected_text == "Figure8")
    {
      return StartNavigationActionGoal::FIGURE8;
    }
    return StartNavigationActionGoal::SQUARE; // Default to square
  }

  void NavigationPanel::startNavigationFeedbackCallback(
      const GoalHandleStartNavigation::SharedPtr &,
      const std::shared_ptr<const StartNavigationAction::Feedback> feedback)
  {

    _status_label->setText(QString("Status: Starting navigation - %1").arg(QString::fromStdString(feedback->status)));
  }

  void NavigationPanel::startNavigationResultCallback(
      const GoalHandleStartNavigation::WrappedResult &result)
  {
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(_client_node->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(_client_node->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(_client_node->get_logger(), "Unknown result code");
      return;
    }
    auto goal_result = result.result;

    _status_label->setText(QString("Status: %1").arg(QString::fromStdString(goal_result->message)));
  }

  void NavigationPanel::startNavigationGoalResponseCallback(
      const GoalHandleStartNavigation::SharedPtr &goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(_client_node->get_logger(), "Goal was rejected by server");
    }
  }

} // namespace localization_kf_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(localization_kf_rviz_plugin::NavigationPanel, rviz_common::Panel)
