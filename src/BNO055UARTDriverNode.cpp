#include <ros/ros.h>

#include "BNO055UARTDriver.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "bno055_uart_driver");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");

    bno055_uart_driver::BNO055UARTDriver communication(nh_);
    communication.spin();
}
