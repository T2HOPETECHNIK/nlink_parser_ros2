#include "init.h"

#include "nutils.h"
#include "protocols.h"

#define ARRAY_ASSIGN(DEST, SRC)                                                \
  for (size_t _CNT = 0; _CNT < sizeof(SRC) / sizeof(SRC[0]); ++_CNT)           \
  {                                                                            \
    DEST[_CNT] = SRC[_CNT];                                                    \
  }

namespace linktrack
{
  anchorframe0 g_msg_anchorframe0;
  tagframe0 g_msg_tagframe0;
  nodeframe0 g_msg_nodeframe0;
  nodeframe1 g_msg_nodeframe1;
  nodeframe2 g_msg_nodeframe2;
  nodeframe3 g_msg_nodeframe3;
  nodeframe5 g_msg_nodeframe5;
  nodeframe6 g_msg_nodeframe6;


  Init::Init(NProtocolExtracter *protocol_extraction, serial::Serial *serial) : Node("linktrack_ros2")
  {
    this->declare_parameter("linktrack_publish_interval", 2.);
    serial_ = serial;
    protocol_extraction_ = protocol_extraction;
    initDataTransmission();
    initAnchorFrame0(protocol_extraction);
    initTagFrame0(protocol_extraction);
    initNodeFrame0(protocol_extraction);
    initNodeFrame1(protocol_extraction);
    initNodeFrame2(protocol_extraction);
    initNodeFrame3(protocol_extraction);
    initNodeFrame5(protocol_extraction);
    initNodeFrame6(protocol_extraction);

    rclcpp::QoS qos(rclcpp::KeepLast(200));
    pub_anchor_frame0_= create_publisher<anchorframe0>("nlink_linktrack_anchorframe0", qos);
    pub_tag_frame0_= create_publisher<tagframe0>("nlink_linktrack_tagframe0", qos);
    pub_node_frame0_= create_publisher<nodeframe0>("nlink_linktrack_nodeframe0", qos);
    pub_node_frame1_= create_publisher<nodeframe1>("nlink_linktrack_nodeframe1", qos);
    pub_node_frame2_= create_publisher<nodeframe2>("nlink_linktrack_nodeframe2", qos);
    pub_node_frame3_= create_publisher<nodeframe3>("nlink_linktrack_nodeframe3", qos);
    pub_node_frame5_= create_publisher<nodeframe5>("nlink_linktrack_nodeframe5", qos);
    pub_node_frame6_= create_publisher<nodeframe6>("nlink_linktrack_nodeframe6", qos);
    std::cout<<"here"<<1000.*this->get_parameter("linktrack_publish_interval").as_double()<<std::endl;
    int pub_interval = (int)(1000.*(this->get_parameter("linktrack_publish_interval").as_double()));
    RCLCPP_INFO(this->get_logger(),"Parameter [linktrack_publish_interval] set to [%d] milliseconds",pub_interval);
    serial_read_timer_ =  this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Init::serialReadTimer, this));
    nodeframe_publisher_ =  this->create_wall_timer(std::chrono::milliseconds(pub_interval), std::bind(&Init::nodeFramePublisher, this));
    RCLCPP_INFO(this->get_logger(),"Initialized linktrack");
  }

  void Init::nodeFramePublisher(){
    pub_anchor_frame0_->publish(this->buffer_msg_anchorframe0_);
    pub_tag_frame0_->publish(this->buffer_msg_tagframe0_);
    pub_node_frame0_->publish(this->buffer_msg_nodeframe0_);
    pub_node_frame1_->publish(this->buffer_msg_nodeframe1_);
    pub_node_frame2_->publish(this->buffer_msg_nodeframe2_);
    pub_node_frame3_->publish(this->buffer_msg_nodeframe3_);
    pub_node_frame5_->publish(this->buffer_msg_nodeframe5_);
    pub_node_frame6_->publish(this->buffer_msg_nodeframe6_);
  }

  void Init::serialReadTimer(){
    auto available_bytes = this->serial_->available();
    std::string str_received;
    if (available_bytes)
    {
      this->serial_->read(str_received, available_bytes);
      this->protocol_extraction_->AddNewData(str_received);
    }
  }

  void Init::initDataTransmission()
  {
    auto callback = [this](const std_msgs::msg::String::SharedPtr msg) -> void {
    if (this->serial_)
      this->serial_->write(msg->data);
    };
    dt_sub_ =
        create_subscription<std_msgs::msg::String>("nlink_linktrack_data_transmission", 1000, callback);
  }

  void Init::initAnchorFrame0(NProtocolExtracter *protocol_extraction)
  {
    auto protocol = new NLT_ProtocolAnchorFrame0;
    protocol_extraction->AddProtocol(protocol);
    protocol->SetHandleDataCallback([=] {

      auto data = nlt_anchorframe0_.result;
      g_msg_anchorframe0.role = data.role;
      g_msg_anchorframe0.id = data.id;
      g_msg_anchorframe0.voltage = data.voltage;
      g_msg_anchorframe0.local_time = data.local_time;
      g_msg_anchorframe0.system_time = data.system_time;
      auto &msg_nodes = g_msg_anchorframe0.nodes;
      msg_nodes.clear();
      decltype(g_msg_anchorframe0.nodes)::value_type msg_node;
      for (size_t i = 0, icount = data.valid_node_count; i < icount; ++i)
      {
        auto node = data.nodes[i];
        msg_node.role = node->role;
        msg_node.id = node->id;
        ARRAY_ASSIGN(msg_node.pos_3d, node->pos_3d)
        ARRAY_ASSIGN(msg_node.dis_arr, node->dis_arr)
        msg_nodes.push_back(msg_node);
      }
      // pub_anchor_frame0_->publish(g_msg_anchorframe0);
      buffer_msg_anchorframe0_ = g_msg_anchorframe0;
    });
  }

  void Init::initTagFrame0(NProtocolExtracter *protocol_extraction)
  {
    auto protocol = new NLT_ProtocolTagFrame0;
    protocol_extraction->AddProtocol(protocol);
    protocol->SetHandleDataCallback([=] {

      const auto &data = g_nlt_tagframe0.result;
      auto &msg_data = g_msg_tagframe0;

      msg_data.role = data.role;
      msg_data.id = data.id;
      msg_data.local_time = data.local_time;
      msg_data.system_time = data.system_time;
      msg_data.voltage = data.voltage;
      ARRAY_ASSIGN(msg_data.pos_3d, data.pos_3d)
      ARRAY_ASSIGN(msg_data.eop_3d, data.eop_3d)
      ARRAY_ASSIGN(msg_data.vel_3d, data.vel_3d)
      ARRAY_ASSIGN(msg_data.dis_arr, data.dis_arr)
      ARRAY_ASSIGN(msg_data.imu_gyro_3d, data.imu_gyro_3d)
      ARRAY_ASSIGN(msg_data.imu_acc_3d, data.imu_acc_3d)
      ARRAY_ASSIGN(msg_data.angle_3d, data.angle_3d)
      ARRAY_ASSIGN(msg_data.quaternion, data.quaternion)
      // pub_tag_frame0_->publish(msg_data);
      buffer_msg_tagframe0_ = msg_data;
    });
  }

  void Init::initNodeFrame0(NProtocolExtracter *protocol_extraction)
  {
    auto protocol = new NLT_ProtocolNodeFrame0;
    protocol_extraction->AddProtocol(protocol);
    protocol->SetHandleDataCallback([=] {

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
      // pub_node_frame0_->publish(msg_data);
      buffer_msg_nodeframe0_ = msg_data;
    });
  }

  void Init::initNodeFrame1(NProtocolExtracter *protocol_extraction)
  {
    auto protocol = new NLT_ProtocolNodeFrame1;
    protocol_extraction->AddProtocol(protocol);
    protocol->SetHandleDataCallback([=] {

      const auto &data = g_nlt_nodeframe1.result;
      auto &msg_data = g_msg_nodeframe1;
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
        ARRAY_ASSIGN(msg_node.pos_3d, node->pos_3d)
      }
      // pub_node_frame1_->publish(msg_data);
      buffer_msg_nodeframe1_ = msg_data;
    });
  }

  void Init::initNodeFrame2(NProtocolExtracter *protocol_extraction)
  {
    auto protocol = new NLT_ProtocolNodeFrame2;
    protocol_extraction->AddProtocol(protocol);
    protocol->SetHandleDataCallback([=] {

      const auto &data = g_nlt_nodeframe2.result;
      auto &msg_data = g_msg_nodeframe2;
      auto &msg_nodes = msg_data.nodes;

      msg_data.role = data.role;
      msg_data.id = data.id;
      msg_data.local_time = data.local_time;
      msg_data.system_time = data.system_time;
      msg_data.voltage = data.voltage;
      ARRAY_ASSIGN(msg_data.pos_3d, data.pos_3d)
      ARRAY_ASSIGN(msg_data.eop_3d, data.eop_3d)
      ARRAY_ASSIGN(msg_data.vel_3d, data.vel_3d)
      ARRAY_ASSIGN(msg_data.imu_gyro_3d, data.imu_gyro_3d)
      ARRAY_ASSIGN(msg_data.imu_acc_3d, data.imu_acc_3d)
      ARRAY_ASSIGN(msg_data.angle_3d, data.angle_3d)
      ARRAY_ASSIGN(msg_data.quaternion, data.quaternion)

      msg_nodes.resize(data.valid_node_count);
      for (size_t i = 0; i < data.valid_node_count; ++i)
      {
        auto &msg_node = msg_nodes[i];
        auto node = data.nodes[i];
        msg_node.id = node->id;
        // std::cout<<"node_id: "<<node->id<<'\n';
        // RCLCPP_INFO(this->get_logger(),"node id : [%d]", node->id);
        msg_node.role = node->role;
        msg_node.dis = node->dis;
        msg_node.fp_rssi = node->fp_rssi;
        msg_node.rx_rssi = node->rx_rssi;
      }
      // pub_node_frame2_->publish(msg_data);
      this->buffer_msg_nodeframe2_ = msg_data;
    });
  }

  void Init::initNodeFrame3(NProtocolExtracter *protocol_extraction)
  {
    auto protocol = new NLT_ProtocolNodeFrame3;
    protocol_extraction->AddProtocol(protocol);
    protocol->SetHandleDataCallback([=] {
      // if (!publishers_[protocol])
      // {
      //   auto topic = "nlink_linktrack_nodeframe3";
      //   rclcpp::QoS qos(rclcpp::KeepLast(200));
      //   publishers_[protocol] =
      //          create_publisher<nlink_parser_ros2_interfaces::msg::LinktrackNodeframe3>(topic, qos);
      //   TopicAdvertisedTip(topic);
      // }
      const auto &data = g_nlt_nodeframe3.result;
      auto &msg_data = g_msg_nodeframe3;
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
        msg_node.fp_rssi = node->fp_rssi;
        msg_node.rx_rssi = node->rx_rssi;
      }
      // pub_node_frame3_->publish(msg_data);
      buffer_msg_nodeframe3_ = msg_data;
    });
  }

  void Init::initNodeFrame5(NProtocolExtracter *protocol_extraction)
  {
    auto protocol = new NLT_ProtocolNodeFrame5;
    protocol_extraction->AddProtocol(protocol);
    protocol->SetHandleDataCallback([=] {
      // if (!publishers_[protocol])
      // {
      //   auto topic = "nlink_linktrack_nodeframe5";
      //      rclcpp::QoS qos(rclcpp::KeepLast(200));
      //   publishers_[protocol] =
      //       create_publisher<nlink_parser_ros2_interfaces::msg::LinktrackNodeframe5>(topic, qos);
      //   TopicAdvertisedTip(topic);
      // }
      const auto &data = g_nlt_nodeframe5.result;
      auto &msg_data = g_msg_nodeframe5;
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
        msg_node.fp_rssi = node->fp_rssi;
        msg_node.rx_rssi = node->rx_rssi;
      }
      // pub_node_frame5_->publish(msg_data);
      buffer_msg_nodeframe5_ = msg_data;
    });
  }

  void Init::initNodeFrame6(NProtocolExtracter *protocol_extraction)
  {
    auto protocol = new NLT_ProtocolNodeFrame6;
    protocol_extraction->AddProtocol(protocol);
    protocol->SetHandleDataCallback([=] {
      // if (!publishers_[protocol])
      // {
      //   auto topic = "nlink_linktrack_nodeframe6";
      //   rclcpp::QoS qos(rclcpp::KeepLast(200));
      //   publishers_[protocol] =
      //       create_publisher<nlink_parser_ros2_interfaces::msg::LinktrackNodeframe6>(topic, qos);
      //   TopicAdvertisedTip(topic);
      //   ;
      // }
      const auto &data = g_nlt_nodeframe6.result;
      auto &msg_data = g_msg_nodeframe6;
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
      // pub_node_frame6_->publish(msg_data);
      buffer_msg_nodeframe6_ = msg_data;
    });
  }

} // namespace linktrack
