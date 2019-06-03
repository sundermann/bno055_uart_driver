#include <memory>
#include "BNO055UARTDriver.h"

namespace bno055_uart_driver {

BNO055UARTDriver::BNO055UARTDriver(ros::NodeHandle &nh) {
    device = nh.param<std::string>("device", "/dev/ttyUSB1");
    baudrate = nh.param("baud", 115200);

    imuPublisher = nh.advertise<sensor_msgs::Imu>("imu", 1);
    imuTemperaturePublisher = nh.advertise<sensor_msgs::Temperature>("imu/temperature", 1);
}

void BNO055UARTDriver::spin() {
    bool connected = false;

    ros::Rate r(100);
    while (ros::ok()) {
        try {
            if (!connected) {
                serial = std::make_unique<serial::Serial>(device, baudrate, serial::Timeout(200, 200, 0, 200, 0));
                connected = serial->isOpen();

                if (connected) {
                    bno055_uart_init(&bno055, serial.get());

                    bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
                    bno055_set_accel_unit(BNO055_ACCEL_UNIT_MSQ);
                    bno055_set_clk_src(BNO055_BIT_ENABLE);
                    bno055_set_euler_unit(BNO055_EULER_UNIT_RAD);
                    bno055_set_gyro_unit(BNO055_GYRO_UNIT_RPS);
                    bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
                    bno055_set_operation_mode(BNO055_OPERATION_MODE_IMUPLUS);
                }
            }

        } catch (const std::exception& exception) {
            connected = false;
            ROS_ERROR_THROTTLE(1, "Could not connect to bno055 %s", exception.what());
        }

        if (serial->isOpen()) {
            try {
                readIMUData();
            } catch (const std::exception& exception) {
                connected = false;
                ROS_ERROR_THROTTLE(1, "Could read from bno055 %s", exception.what());
            }
        }

        ros::spinOnce();
        r.sleep();
    }
}

void BNO055UARTDriver::readIMUData() {
    sensor_msgs::Imu imuMsg;
    auto now = ros::Time::now();
    imuMsg.header.frame_id = "imu";
    imuMsg.header.stamp = now;

    BNO055Register data {};
    bno055_read_register(BNO055_CHIP_ID_ADDR, (u8*)(&data), sizeof(data));

    sensor_msgs::Temperature temperatureMsg;
    temperatureMsg.header.stamp = now;
    temperatureMsg.header.frame_id = "imu";
    temperatureMsg.temperature = data.temperature / (double)BNO055_TEMP_DIV_CELSIUS;
    imuTemperaturePublisher.publish(temperatureMsg);

    // map from NED to ENU
    imuMsg.orientation.w = data.orientation.w / 16384.0;
    imuMsg.orientation.x = data.orientation.x / 16384.0;
    imuMsg.orientation.y = data.orientation.y / 16384.0;
    imuMsg.orientation.z = data.orientation.z / 16384.0;

    imuMsg.angular_velocity.x = data.gyroscope.x / BNO055_GYRO_DIV_RPS;
    imuMsg.angular_velocity.y = data.gyroscope.y / BNO055_GYRO_DIV_RPS;
    imuMsg.angular_velocity.z = data.gyroscope.z / BNO055_GYRO_DIV_RPS;

    imuMsg.linear_acceleration.x = data.linearAcceleration.x / BNO055_LINEAR_ACCEL_DIV_MSQ;
    imuMsg.linear_acceleration.y = data.linearAcceleration.y / BNO055_LINEAR_ACCEL_DIV_MSQ;
    imuMsg.linear_acceleration.z = data.linearAcceleration.z / BNO055_LINEAR_ACCEL_DIV_MSQ;

    auto system = (data.calibration_status >> 6) & 0x3;
    auto gyro = (data.calibration_status >> 4) & 0x3;
    auto accel = (data.calibration_status >> 2) & 0x3;
    auto magneto = data.calibration_status & 0x3;

    auto linearAccelerationStdDev = (400 / 1000000.0) * 9.807;
    auto angularVelocityStdDev = 0.05 * (M_PI / 180.0);
    auto pitchRollStdDev =  1.0 * (M_PI / 180.0);
    auto yawStdDev =  5.0 * (M_PI / 180.0);

    imuMsg.linear_acceleration_covariance[0] = linearAccelerationStdDev * linearAccelerationStdDev;
    imuMsg.linear_acceleration_covariance[4] = linearAccelerationStdDev * linearAccelerationStdDev;
    imuMsg.linear_acceleration_covariance[8] = linearAccelerationStdDev * linearAccelerationStdDev;

    imuMsg.angular_velocity_covariance[0] = angularVelocityStdDev * angularVelocityStdDev;
    imuMsg.angular_velocity_covariance[4] = angularVelocityStdDev * angularVelocityStdDev;
    imuMsg.angular_velocity_covariance[8] = angularVelocityStdDev * angularVelocityStdDev;

    imuMsg.orientation_covariance[0] = pitchRollStdDev * pitchRollStdDev;
    imuMsg.orientation_covariance[4] = pitchRollStdDev * pitchRollStdDev;
    imuMsg.orientation_covariance[8] = yawStdDev * yawStdDev;

    imuPublisher.publish(imuMsg);
}
}
