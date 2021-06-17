#include <math.h> 

#include "rclcpp/rclcpp.hpp"

#include <r2r2_jedi_force_sensor_interfaces/msg/robot_presence.hpp>
#include <r2r2_jedi_force_sensor_interfaces/msg/robot_presence_array.hpp>
#include <nlink_parser_ros2_interfaces/msg/linktrack_nodeframe2.hpp>

using robot_presence_msg = r2r2_jedi_force_sensor_interfaces::msg::RobotPresence;
using robot_presence_array_msg = r2r2_jedi_force_sensor_interfaces::msg::RobotPresenceArray;
using linktrack_msg = nlink_parser_ros2_interfaces::msg::LinktrackNodeframe2;

class MsgConverter : public rclcpp::Node
{
public:
   explicit MsgConverter()
   : Node("nlink_to_r2r2_msg_converter")
    {

        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
        auto callback =
            [this, clock](const linktrack_msg::SharedPtr msg) -> void
            {
                robot_presence_array_msg robot_collection;
                for(int i=0; i<msg->nodes.size();i++){
                    robot_presence_msg robot;
                    robot.header.frame_id=this->get_parameter("frame_id").value_to_string();
                    robot.header.stamp = clock->now();
                    robot.mode = robot.RANGE_ONLY;
                    robot.range_min = (float)this->get_parameter("range_min").as_double();
                    robot.range_max = (float)this->get_parameter("range_max").as_double();
                    robot.range = msg->nodes[i].dis;
                    robot.id = std::to_string(msg->nodes[i].id);
                    robot_collection.robots.push_back(robot);

                    this->robot_presence_pub_->publish(robot_collection);
                    RCLCPP_INFO(this->get_logger(), "I heard tag: %s at distance %f", robot.id.c_str(), robot.range );
                }
            };

        this->declare_parameter("frame_id", "/map");
        this->declare_parameter("range_min", 0.);
        this->declare_parameter("range_max", 60.);
        // auto frame_id = this->get_parameter("frame_id");
        // std::cout<<frame_id.value<<'\n';
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        linktrack_sub_ = create_subscription<linktrack_msg>("nlink_linktrack_nodeframe2", 10, callback);
        robot_presence_pub_ = create_publisher<robot_presence_array_msg>("robot_presence", qos);
    }

private:
    rclcpp::Subscription<linktrack_msg>::SharedPtr linktrack_sub_;
    rclcpp::Publisher<robot_presence_array_msg>::SharedPtr robot_presence_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MsgConverter>());
    rclcpp::shutdown();
    return 0;
}

