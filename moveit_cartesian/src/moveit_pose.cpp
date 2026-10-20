#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>

class CartesianPoseSubscriber : public rclcpp::Node
{
public:
  CartesianPoseSubscriber()
  : Node("cartesian_pose_subscriber")
  {
    // Initialize MoveGroupInterface
    move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      std::shared_ptr<rclcpp::Node>(this), "panda_arm");

    // Set solver (PRM seems to work best with collision, and is quite fast)
    move_group->setPlannerId("PRMstarkConfigDefault");
    move_group->setPlanningTime(15.0);

    // Make floor
    moveit_msgs::msg::CollisionObject floor;
    floor.header.frame_id = move_group->getPlanningFrame();  // typically panda_link0
    floor.id = "floor";

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {4.0, 4.0, 0.10};  // x, y, z size

    geometry_msgs::msg::Pose pose;
    pose.position.z = -0.05;             // top face is at z=0.15
    pose.orientation.w = 1.0;

    floor.primitives.push_back(box);
    floor.primitive_poses.push_back(pose);
    floor.operation = moveit_msgs::msg::CollisionObject::ADD;

    if (!planning_scene_interface.applyCollisionObject(floor)) {
      RCLCPP_ERROR(get_logger(), "Failed to add floor to the planning scene");
    }

    moveit_msgs::msg::AttachedCollisionObject acm;
    acm.link_name = "panda_link0";
    acm.object.id = "floor";
    acm.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    acm.touch_links = {"panda_link0", "panda_link1"};
    planning_scene_interface.applyAttachedCollisionObject(acm);



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
  // Planning for floor, construction redundant
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
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
      SEND PLAN
    */
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    auto success = move_group->plan(my_plan);// == moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(
      this->get_logger(),
      "Plan Result: code=%d, success=%s",
      success.val,
      success == moveit::core::MoveItErrorCode::SUCCESS ? "true" : "false");

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
