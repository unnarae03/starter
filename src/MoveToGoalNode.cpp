#include "MoveToGoalNode.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

MoveToGoalNode::MoveToGoalNode(const std::string& name, const BT::NodeConfiguration& config)
: BT::AsyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("move_to_goal_bt_node");
  action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");

  if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    throw std::runtime_error("Action server not available after waiting");
  }
}

BT::PortsList MoveToGoalNode::providedPorts()
{
  return {
    BT::InputPort<double>("x"),
    BT::InputPort<double>("y"),
    BT::InputPort<double>("theta")
  };
}

BT::NodeStatus MoveToGoalNode::tick()
{
  double x, y, theta;
  if (!getInput("x", x) || !getInput("y", y) || !getInput("theta", theta)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing input port");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.header.stamp = node_->get_clock()->now();
  goal_pose.pose.position.x = x;
  goal_pose.pose.position.y = y;

  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  goal_pose.pose.orientation = tf2::toMsg(q);

  NavigateToPose::Goal goal_msg;
  goal_msg.pose = goal_pose;

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  auto future_goal = action_client_->async_send_goal(goal_msg, send_goal_options);
  auto goal_handle = future_goal.get();

  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected");
    return BT::NodeStatus::FAILURE;
  }

  auto result_future = action_client_->async_get_result(goal_handle);
  auto result = result_future.get();

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

void MoveToGoalNode::halt()
{
  // 중단 시 특별한 처리 없음
}
