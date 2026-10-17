#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class TrajectoryPoseSubscriber : public rclcpp::Node
{
public:
  TrajectoryPoseSubscriber()
  : Node("cartesian_pose_subscriber")
  {
    // Initialize MoveGroupInterface
    move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      std::shared_ptr<rclcpp::Node>(this), "panda_arm");

    // Subscribed topic name
    subscription = this->create_subscription<geometry_msgs::msg::Pose>(
      "/target_cartesian_pose", 10,
      std::bind(&TrajectoryPoseSubscriber::poseCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Listening for Cartesian poses on /target_cartesian_pose...");
  }

private:
  static constexpr double eef_step_ = 0.01;
  static constexpr double jump_threshold_ = 1.5;
  static constexpr double minimum_fraction_ = 0.5;

  // MoveGroup and Topic variables
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription;
  
  // Called whenever a new pose is given
  void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    // Log the pose location (orientation is also included, just not logged)
    RCLCPP_INFO(this->get_logger(), "Received target pose: x=%.2f, y=%.2f, z=%.2f",
                msg->position.x, msg->position.y, msg->position.z);

    // Get start state as pose
    geometry_msgs::msg::PoseStamped current_pose_stamped = move_group->getCurrentPose();
    geometry_msgs::msg::Pose current_pose = current_pose_stamped.pose;
    // Add it to a list of waypoints
    std::vector<geometry_msgs::msg::Pose> waypoints{current_pose};
    // Add the target as a waypoint
    waypoints.push_back(*msg);
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = move_group->computeCartesianPath(
      waypoints, eef_step_, jump_threshold_, trajectory, true);

    if (fraction < minimum_fraction_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Cartesian path is incomplete (%.1f%%); refusing to execute partial trajectory.",
        fraction * 100.0);
      return;
    }

    const auto result = move_group->execute(trajectory);
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Cartesian trajectory execution failed.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Cartesian trajectory executed successfully.");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryPoseSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
