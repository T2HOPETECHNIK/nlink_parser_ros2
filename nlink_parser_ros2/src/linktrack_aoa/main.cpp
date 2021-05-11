#include <rclcpp/rclcpp.hpp>

#include "init.h"
#include "init_serial.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<MinimalPublisher>());
  
  // return 0;

  // ros::init(argc, argv, "linktrack_aoa");
  // ros::NodeHandle nh;
  serial::Serial serial;
  initSerial(&serial);
  NProtocolExtracter protocol_extraction;
  auto aoaInit = std::make_shared<linktrack_aoa::Init>(&protocol_extraction, &serial);
  // ros::Rate loop_rate(1000);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(aoaInit);
  rclcpp::Rate loop_rate(1000);
  while (rclcpp::ok())
  {
    auto available_bytes = serial.available();
    std::string str_received;
    if (available_bytes)
    {
      serial.read(str_received, available_bytes);
      protocol_extraction.AddNewData(str_received);
    }
    executor.spin_once();
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
