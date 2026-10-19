#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

class CartesianPoseSubscriber : public rclcpp::Node
{
public:
  CartesianPoseSubscriber()
  : Node("cartesian_pose_subscriber")
  {
    // Initialize MoveGroupInterface
    move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      std::shared_ptr<rclcpp::Node>(this), "panda_arm");

    // Subscribed topic name
    subscription = this->create_subscription<geometry_msgs::msg::Pose>(
      "/target_cartesian_pose", 10,
      std::bind(&CartesianPoseSubscriber::poseCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Listening for Cartesian poses on /target_cartesian_pose...");
  }

private:
  // MoveGroup and Topic variables
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription;
  
  // Called whenever a new pose is given
  void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    /*
      POSE DEFINITION
    */

    // Log the recevied pose
    RCLCPP_INFO(this->get_logger(), "Received target pose: x=%.2f, y=%.2f, z=%.2f",
                msg->position.x, msg->position.y, msg->position.z);

    if (msg->position.z <= 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Pose planning halted, z-value %.2f <= 0", msg->position.z);
      return;
    }

    // Save the pose as the target
    move_group->setPoseTarget(*msg);

    /*
      PATH CONSTRAINTS
    */

    // Define path constraints
    moveit_msgs::msg::Constraints path_constraints;
    moveit_msgs::msg::PositionConstraint position_constraint;

    // Define world and target links
    position_constraint.header.frame_id = "panda_link0";
    position_constraint.link_name = "panda_hand_tcp";
    // FULL WEIGHT
    position_constraint.weight = 1.0;

    // Make a box-shaped region that the arm can move in
    shape_msgs::msg::SolidPrimitive region;
    region.type = shape_msgs::msg::SolidPrimitive::BOX;
    region.dimensions = {10.0, 10.0, 10.0};

    // Center the box
    geometry_msgs::msg::Pose region_pose;
    region_pose.orientation.w = 1.0;
    region_pose.position.z = -0.1;  // Center of the z range [0.12, 10.12]

    // Store primatives as constraints
    position_constraint.constraint_region.primitives.push_back(region);
    position_constraint.constraint_region.primitive_poses.push_back(region_pose);
    path_constraints.position_constraints.push_back(position_constraint);

    // Push the constraintsd to the move group
    move_group->setPathConstraints(path_constraints);

    /*
      SEND PLAN
    */
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group->clearPathConstraints();

    if (success) {
      RCLCPP_INFO(this->get_logger(), "Plan succeeded! Executing trajectory...");
      move_group->execute(my_plan);
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
