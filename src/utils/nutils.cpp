#include <iostream>
#include <nutils.h>

#include <rclcpp/rclcpp.hpp>

void TopicAdvertisedTip(const char *topic)
{
  // RCLCPP_INFO("%s has been advertised,use 'rostopic "
          //  "echo /%s' to view the data",
          //  topic, topic);
    std::cout<<topic << " has been advertised"<<std::endl;
          //  "echo /%s' to view the data",
          //  topic, topic
}
