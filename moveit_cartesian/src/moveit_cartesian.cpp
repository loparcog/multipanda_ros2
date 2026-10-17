#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class CartesianPoseSubscriber : public rclcpp::Node
{
public:
  CartesianPoseSubscriber()
  : Node("cartesian_pose_subscriber")
  {
    // Initialize MoveGroupInterface
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      std::shared_ptr<rclcpp::Node>(this), "panda_arm");

    // Subscribed topic name
    subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/target_cartesian_pose", 10,
      std::bind(&CartesianPoseSubscriber::poseCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Listening for Cartesian poses on /target_cartesian_pose...");
  }

private:
  // MoveGroup and Topic variables
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
  
  // Called whenever a new pose is given
  void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    // Log the pose
    RCLCPP_INFO(this->get_logger(), "Received target pose: x=%.2f, y=%.2f, z=%.2f",
                msg->position.x, msg->position.y, msg->position.z);

    // Set the pose
    move_group_->setPoseTarget(*msg);

    // Check for success and log accordingly
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      RCLCPP_INFO(this->get_logger(), "Plan succeeded! Executing trajectory...");
      move_group_->execute(my_plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Motion planning failed for the target pose!");
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CartesianPoseSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
