#ifndef LINKTRACKAOAINIT_H
#define LINKTRACKAOAINIT_H

#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

#include "../linktrack/protocols.h"
#include "nlink_protocol.h"

#include "std_msgs/msg/string.hpp"
#include <nlink_parser_ros2_interfaces/msg/linktrack_aoa_nodeframe0.hpp>
#include <nlink_parser_ros2_interfaces/msg/linktrack_nodeframe0.hpp>

#include "protocol_extracter/nprotocol_extracter.h"

namespace linktrack_aoa
{
  class Init : public rclcpp::Node 
  {
  public:
    explicit Init(NProtocolExtracter *protocol_extraction,
                  serial::Serial *serial);

  private:
    rclcpp::TimerBase::SharedPtr serial_read_timer_;
    serial::Serial *g_serial;
    NProtocolExtracter* protocol_extraction_;
    void DTCallback(const std_msgs::msg::String::SharedPtr msg);
    void serialReadTimer();
    void initDataTransmission();
    void initNodeFrame0(NProtocolExtracter *protocol_extraction);
    void InitAoaNodeFrame0(NProtocolExtracter *protocol_extraction);
    std::unordered_map<NProtocolBase *, rclcpp::Publisher<nlink_parser_ros2_interfaces::msg::LinktrackNodeframe0>::SharedPtr> publishers_;
    std::unordered_map<NProtocolBase *, rclcpp::Publisher<nlink_parser_ros2_interfaces::msg::LinktrackAoaNodeframe0>::SharedPtr> publishers_aoa_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dt_sub_;
  };
} // namespace linktrack_aoa

#endif // LINKTRACKAOAINIT_H
