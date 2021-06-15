#include "init.h"
#include "init_serial.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // std::cout<<sizeof(argv)<<" dfg " <<argv[1]<<'\n';
  serial::Serial serial;
  initSerial(&serial, argv[1]);
  NProtocolExtracter protocol_extraction;

  auto aoaInit = std::make_shared<linktrack_aoa::Init>(&protocol_extraction, &serial);

  rclcpp::spin(aoaInit);

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
