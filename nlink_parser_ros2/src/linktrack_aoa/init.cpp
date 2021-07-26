#include "init.h"

#include "nlink_unpack/nlink_linktrack_aoa_nodeframe0.h"
#include "nlink_unpack/nlink_linktrack_nodeframe0.h"
#include "nutils.h"

class NLTAoa_ProtocolNodeFrame0 : public NLinkProtocolVLength
{
public:
  NLTAoa_ProtocolNodeFrame0();

protected:
  void UnpackFrameData(const uint8_t *data) override;
};

NLTAoa_ProtocolNodeFrame0::NLTAoa_ProtocolNodeFrame0()
    : NLinkProtocolVLength(
          true, g_nltaoa_nodeframe0.fixed_part_size,
          {g_nltaoa_nodeframe0.frame_header, g_nltaoa_nodeframe0.function_mark})
{
}

void NLTAoa_ProtocolNodeFrame0::UnpackFrameData(const uint8_t *data)
{
  g_nltaoa_nodeframe0.UnpackData(data, length());
}

namespace linktrack_aoa
{

  nlink_parser_ros2_interfaces::msg::LinktrackNodeframe0 g_msg_nodeframe0;
  nlink_parser_ros2_interfaces::msg::LinktrackAoaNodeframe0 g_msg_aoa_nodeframe0;

  Init::Init(NProtocolExtracter *protocol_extraction, serial::Serial *serial) : Node("linktrack_aoa_ros2")
  {
    g_serial = serial;
    protocol_extraction_ = protocol_extraction;

    this->declare_parameter("pub_frequency",5.);
    initDataTransmission();
    initNodeFrame0(protocol_extraction);
    InitAoaNodeFrame0(protocol_extraction);
    float pub_interval = 1000./this->get_parameter("pub_frequency").as_double();
    serial_read_timer_ =  this->create_wall_timer(std::chrono::milliseconds((int)pub_interval), std::bind(&Init::serialReadTimer, this));
    RCLCPP_INFO(this->get_logger(),"Initialized linktrack AoA");
  }

  void Init::serialReadTimer(){
    auto available_bytes = this->g_serial->available();
    std::string str_received;
    if (available_bytes)
    {
      this->g_serial->read(str_received, available_bytes);
      this->protocol_extraction_->AddNewData(str_received);
    }
  }

  void Init::initDataTransmission()
  {
    auto callback = [this](const std_msgs::msg::String::SharedPtr msg) -> void {
      if (this->g_serial){
        this->g_serial->write(msg->data);
      }
    };
    dt_sub_ =
        create_subscription<std_msgs::msg::String>("nlink_linktrack_data_transmission", 1000, callback);
  }

  void Init::initNodeFrame0(NProtocolExtracter *protocol_extraction)
  {
    auto protocol = new NLT_ProtocolNodeFrame0;
    protocol_extraction->AddProtocol(protocol);
    protocol->SetHandleDataCallback([=] {
      if (!publishers_[protocol])
      {
        auto topic = "nlink_linktrack_nodeframe0";
        rclcpp::QoS qos(rclcpp::KeepLast(200));
        publishers_[protocol] =
            create_publisher<nlink_parser_ros2_interfaces::msg::LinktrackNodeframe0>(topic, qos);
      }
      const auto &data = g_nlt_nodeframe0.result;
      auto &msg_data = g_msg_nodeframe0;
      auto &msg_nodes = msg_data.nodes;

      msg_data.role = data.role;
      msg_data.id = data.id;

      msg_nodes.resize(data.valid_node_count);
      for (size_t i = 0; i < data.valid_node_count; ++i)
      {
        auto &msg_node = msg_nodes[i];
        auto node = data.nodes[i];
        msg_node.id = node->id;
        msg_node.role = node->role;
        msg_node.data.resize(node->data_length);
        memcpy(msg_node.data.data(), node->data, node->data_length);
      }

      publishers_.at(protocol)->publish(msg_data);
    });
  }

  void Init::InitAoaNodeFrame0(NProtocolExtracter *protocol_extraction)
  {
    auto protocol = new NLTAoa_ProtocolNodeFrame0;
    protocol_extraction->AddProtocol(protocol);
    protocol->SetHandleDataCallback([=] {
      if (!publishers_[protocol])
      {
        auto topic = "nlink_linktrack_aoa_nodeframe0";
        rclcpp::QoS qos(rclcpp::KeepLast(200));
        publishers_aoa_[protocol] =
            create_publisher<nlink_parser_ros2_interfaces::msg::LinktrackAoaNodeframe0>(topic, qos);
      }
      const auto &data = g_nltaoa_nodeframe0.result;
      auto &msg_data = g_msg_aoa_nodeframe0;
      auto &msg_nodes = msg_data.nodes;

      msg_data.role = data.role;
      msg_data.id = data.id;
      msg_data.local_time = data.local_time;
      msg_data.system_time = data.system_time;
      msg_data.voltage = data.voltage;

      msg_nodes.resize(data.valid_node_count);
      for (size_t i = 0; i < data.valid_node_count; ++i)
      {
        auto &msg_node = msg_nodes[i];
        auto node = data.nodes[i];
        msg_node.id = node->id;
        msg_node.role = node->role;
        msg_node.dis = node->dis;
        msg_node.angle = node->angle;
        msg_node.fp_rssi = node->fp_rssi;
        msg_node.rx_rssi = node->rx_rssi;
      }
      publishers_aoa_.at(protocol)->publish(msg_data);
    });
  }

} // namespace linktrack_aoa
