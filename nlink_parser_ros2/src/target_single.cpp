#include <math.h> 

#include "rclcpp/rclcpp.hpp"

#include <nlink_parser_ros2_interfaces/msg/linktrack_aoa_nodeframe0.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#define PI 3.14159265

using aoa_nodeframe = nlink_parser_ros2_interfaces::msg::LinktrackAoaNodeframe0;

class Listener : public rclcpp::Node
{
public:
   explicit Listener()
  : Node("nlink_vizualizer")
  {
    
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    auto callback =
      [this, clock](const aoa_nodeframe::SharedPtr msg) -> void
      {
        float val;
        for(int i=0; i<msg->nodes.size();i++){
          geometry_msgs::msg::PoseStamped poseStamped;
          poseStamped.header.frame_id="/map";
          poseStamped.header.stamp = clock->now();

          poseStamped.pose.position.x = msg->nodes[i].dis * cos(msg->nodes[i].angle*PI/180);
          poseStamped.pose.position.y = msg->nodes[i].dis * sin(msg->nodes[i].angle*PI/180);
          val = msg->nodes[i].dis;
          this->globalGoalPoseStampedPub->publish(poseStamped);
        }
        RCLCPP_INFO(this->get_logger(), "I heard: [%f]", val);
      };
    
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    sub_ = create_subscription<aoa_nodeframe>("nlink_linktrack_aoa_nodeframe0", 10, callback);
    globalGoalPoseStampedPub = create_publisher<geometry_msgs::msg::PoseStamped>("uwb_goal", qos);
  }

private:
  rclcpp::Subscription<aoa_nodeframe>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr globalGoalPoseStampedPub;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}

