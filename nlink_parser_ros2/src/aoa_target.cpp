#include <math.h> 
#include <chrono> 
#include <ctime>


#include "rclcpp/rclcpp.hpp"

#include <nlink_parser_ros2_interfaces/msg/linktrack_aoa_nodeframe0.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#define PI 3.14159265

using aoa_nodeframe = nlink_parser_ros2_interfaces::msg::LinktrackAoaNodeframe0;
using namespace std::chrono_literals;
class Listener : public rclcpp::Node
{
public:
   explicit Listener()
  : Node("nlink_vizualizer")
  {
    
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

    this->declare_parameter("node_ns", "linktrack_aoa");
    this->declare_parameter("frame_id", "base_link");
    this->declare_parameter("node_distance", .25);

    auto callback_linktrack_node =
      [this, clock](const aoa_nodeframe::SharedPtr msg) -> void
      {
        if(msg->nodes.size()==0) return;
        geometry_msgs::msg::PoseStamped poseStamped;
        poseStamped.header.frame_id=this->get_parameter("frame_id").as_string();
        poseStamped.header.stamp = clock->now();

        auto dis = msg->nodes[0].dis;
        auto angle = msg->nodes[0].angle;
        float x_offset = 0;
        // if(msg->nodes.size()>1 && msg->nodes[0].dis > msg->nodes[1].dis){
        //   angle = 180-angle ;
        //   dis = msg->nodes[0].angle;
        //   x_offset = this->get_parameter("node_distance").as_double();
        //   RCLCPP_INFO(this->get_logger(), "node 2");
        // } else RCLCPP_INFO(this->get_logger(), "node 1");
        if(msg->nodes.size()>1){
          if (msg->nodes[0].dis > msg->nodes[1].dis){
            angle = 180.-angle;
            // x_offset = this->get_parameter("node_distance").as_double();
            RCLCPP_INFO(this->get_logger(), "node 2");
          } else {
            RCLCPP_INFO(this->get_logger(), "node 1");
          }
        } else {
          if(msg->nodes[0].id==1) {
            angle = 180.-angle;
            // x_offset = this->get_parameter("node_distance").as_double();
            RCLCPP_INFO(this->get_logger(), "node 2 only");
          } else {
            RCLCPP_INFO(this->get_logger(), "node 1 only");
          }
        }
        poseStamped.pose.position.x = dis * cos(angle*PI/180);
        poseStamped.pose.position.x -= x_offset;
        poseStamped.pose.position.y = dis * sin(angle*PI/180);
        this->globalGoalPoseStampedPub->publish(poseStamped);
        
      };

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    //std::string topic = this->get_parameter("node_ns").as_string()+"/nlink_linktrack_aoa_nodeframe0"; 
    std::string topic = "/nlink_linktrack_aoa_nodeframe0"; 
    sub_linktrack_aoa_ = create_subscription<aoa_nodeframe>(topic, 10, callback_linktrack_node);

    globalGoalPoseStampedPub = create_publisher<geometry_msgs::msg::PoseStamped>("global_goal", qos);
  }

private:
  // float range_from_node1, range_from_node2, angle_from_node1, angle_from_node2;

  rclcpp::Subscription<aoa_nodeframe>::SharedPtr sub_linktrack_aoa_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr globalGoalPoseStampedPub;
  rclcpp::TimerBase::SharedPtr pub_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}

