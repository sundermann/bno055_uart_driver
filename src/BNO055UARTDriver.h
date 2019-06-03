#pragma once

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>

#include <serial/serial.h>

#include "bno055_uart.h"

namespace bno055_uart_driver {

struct BNO055Register {
    uint8_t chipId, accelorometerId, magnetometerId, gyroscopeId;
    uint16_t softwareRevision;
    uint8_t bootloaderVersion;
    uint8_t pageId;
    struct acceleration {
        int16_t x, y, z;
    } acceleration;
    struct magnetometer {
        int16_t x, y, z;
    } magnetometer;
    struct gyroscope {
        int16_t x, y, z;
    } gyroscope;
    struct eulerAngles {
        int16_t heading, roll, pitch;
    } eulerAngles;
    struct orientation {
        int16_t w, x, y, z;
    } orientation;
    struct linearAcceleration {
        int16_t x, y, z;
    } linearAcceleration;
    struct gravity {
        int16_t x, y, z;
    } gravity;
    int8_t temperature;
    int8_t calibration_status;
};

class BNO055UARTDriver {
 public:
    explicit BNO055UARTDriver(ros::NodeHandle &nh);

    void spin();
 private:
    void readIMUData();

    ros::Publisher imuPublisher;
    ros::Publisher imuTemperaturePublisher;

    std::string device;
    uint32_t baudrate;
    std::unique_ptr<serial::Serial> serial;

    bno055_t bno055;
};

}