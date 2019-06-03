/*
****************************************************************************
* Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
*
* bno055_support.c
* Date: 2016/03/14
* Revision: 1.0.4 $
*
* Usage: Sensor Driver support file for BNO055 sensor
*
****************************************************************************
* License:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/
/*---------------------------------------------------------------------------*
 Includes
*---------------------------------------------------------------------------*/
#include "bno055_uart.h"

#include <unistd.h>
#include <ros/ros.h>

serial::Serial* connection;

/*--------------------------------------------------------------------------*
*	The following API is used to map the I2C bus read, write, delay and
*	device address with global structure bno055_t
*-------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------*
 *  By using bno055 the following structure parameter can be accessed
 *	Bus write function pointer: BNO055_WR_FUNC_PTR
 *	Bus read function pointer: BNO055_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *--------------------------------------------------------------------------*/
s8 bno055_uart_init(bno055_t* bno055, serial::Serial* serial)
{
	bno055->bus_write = BNO055_I2C_bus_write;
	bno055->bus_read = BNO055_I2C_bus_read;
	bno055->delay_msec = BNO055_delay_msek;
	bno055->dev_addr = BNO055_I2C_ADDR1;

	connection = serial;
	//serial->setTimeout(200, 200, 20, 0,0);

	return bno055_init(bno055);
}

/************** I2C buffer length******/

#define	I2C_BUFFER_LEN 8
#define I2C0 5
/*-------------------------------------------------------------------*
*
*	This is a sample code for read and write the data by using I2C
*	Use either I2C  based on your need
*	The device address defined in the bno055.h file
*
*--------------------------------------------------------------------*/

/*	\Brief: The API is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *   will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	std::vector<u8> array;

	array.emplace_back(0xAA);
	array.emplace_back(0x00);
	array.emplace_back(reg_addr);
	array.emplace_back(cnt);

	for (uint8_t i = 0; i < cnt; i++) {
	    array.emplace_back(reg_data[i]);
	}

    for (u8 data : array) {
        connection->write(&data, 1);
        // The BNO055 needs some time to clear the input buffer so instead of sending all the data at once at full burst
        // we just wait a little in between commands
        usleep(200);
    }

    std::vector<u8> result;

    int iterations = 0;
    while (result.size() < 2) {
        auto read = connection->read(2 - result.size());
        std::copy(read.begin(), read.end(), std::back_inserter(result));
        iterations++;

        if (iterations > 10) {
            ROS_ERROR("Timeout");
            BNO055_reset_connection();
            return BNO055_I2C_bus_write(dev_addr, reg_addr, reg_data, cnt);
        }
    }

    if (result[0] == 0xEE) {
        switch ((WRITE_COMMAND_RESPONSE) result[1]) {
            case WRITE_SUCCESS:
                return BNO055_SUCCESS;
            case WRITE_FAIL:
                ROS_ERROR("Write Fail");
                BNO055_reset_connection();
                return BNO055_I2C_bus_write(dev_addr, reg_addr, reg_data, cnt);
            case REGMAP_INVALID_ADDRESS:
                ROS_ERROR("Invalid address");
                break;
            case REGMAP_WRITE_DISABLED:
                ROS_ERROR("Write disabled");
                break;
            case WRONG_START_BYTE:
                ROS_ERROR("Wrong start byte");
                BNO055_reset_connection();
                return BNO055_I2C_bus_write(dev_addr, reg_addr, reg_data, cnt);
            case BUS_OVER_RUN_ERROR:
                ROS_ERROR("Bus overrun");
                //BNO055_reset_connection();
                return BNO055_I2C_bus_write(dev_addr, reg_addr, reg_data, cnt);
            case MAX_LENGTH_ERROR:
                ROS_ERROR("Max length exceeded");
                break;
            case MIN_LENGTH_ERROR:
                ROS_ERROR("Min length");
                break;
            case RECEIVE_CHARACTER_TIMEOUT:
                ROS_ERROR("Timeout");
                BNO055_reset_connection();
                return BNO055_I2C_bus_write(dev_addr, reg_addr, reg_data, cnt);
            default:
                ROS_ERROR("not handled");
                BNO055_reset_connection();
                return BNO055_I2C_bus_write(dev_addr, reg_addr, reg_data, cnt);
        }

        return BNO055_ERROR;
    } else {
        // We received something else maybe data package?
        BNO055_reset_connection();
        return BNO055_I2C_bus_write(dev_addr, reg_addr, reg_data, cnt);
    }
}

 /*	\Brief: The API is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *  will data is going to be read
 *	\param reg_data : This data read from the sensor,
 *   which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	/* Please take the below API as your reference
	 * for read the data using I2C communication
	 * add your I2C read API here.
	 * "BNO055_iERROR = I2C_WRITE_READ_STRING(DEV_ADDR,
	 * ARRAY, ARRAY, 1, CNT)"
	 * BNO055_iERROR is an return value of SPI write API
	 * Please select your valid return value
     * In the driver BNO055_SUCCESS defined as 0
     * and FAILURE defined as -1
	 */

	std::vector<u8> array;
	array.emplace_back(0xAA);
	array.emplace_back(0x01);
	array.emplace_back(reg_addr);
	array.emplace_back(cnt);

	for (u8 data : array) {
	    connection->write(&data, 1);
	    // The BNO055 needs some time to clear the input buffer so instead of sending all the data at once at full burst
	    // we just wait a little in between commands
	    usleep(200);
	}

	std::vector<u8> result;
	int iterations = 0;
	while (result.size() < 2) {
	    auto read = connection->read(2 - result.size());
        std::copy(read.begin(), read.end(), std::back_inserter(result));
        iterations++;

        if (iterations > 10) {
            BNO055_reset_connection();
            return BNO055_I2C_bus_read(dev_addr, reg_addr, reg_data, cnt);
        }
	}

    int len = result[1];

    if (result[0] == 0xEE) {
        switch ((WRITE_COMMAND_RESPONSE)result[1]) {
            case WRITE_SUCCESS:
                ROS_ERROR("Write Success");
                return BNO055_SUCCESS;
            case WRITE_FAIL:
                ROS_ERROR("Write Fail");
                BNO055_reset_connection();
                return BNO055_I2C_bus_read(dev_addr, reg_addr, reg_data, cnt);
            case REGMAP_INVALID_ADDRESS:
                ROS_ERROR("Invalid address");
                break;
            case REGMAP_WRITE_DISABLED:
                ROS_ERROR("Write disabled");
                break;
            case WRONG_START_BYTE:
                ROS_ERROR("Wrong start byte");
                BNO055_reset_connection();
                return BNO055_I2C_bus_read(dev_addr, reg_addr, reg_data, cnt);
            case BUS_OVER_RUN_ERROR:
                ROS_ERROR("Bus overrun");
                //BNO055_reset_connection();
                return BNO055_I2C_bus_read(dev_addr, reg_addr, reg_data, cnt);
            case MAX_LENGTH_ERROR:
                ROS_ERROR("Max length exceeded");
                break;
            case MIN_LENGTH_ERROR:
                ROS_ERROR("Min length");
                break;
            case RECEIVE_CHARACTER_TIMEOUT:
                ROS_ERROR("Timeout");
                BNO055_reset_connection();
                return BNO055_I2C_bus_read(dev_addr, reg_addr, reg_data, cnt);
            default:
                ROS_ERROR("not handled");
                break;
        }

        return BNO055_ERROR;
    }

    if (result[0] == 0xBB) {
        result.clear();

        iterations = 0;
        while (result.size() < len) {
            auto read = connection->read(len - result.size());
            std::copy(read.begin(), read.end(), std::back_inserter(result));

            iterations++;

            if (iterations > 10) {
                ROS_ERROR("Timeout!");
                BNO055_reset_connection();
                return BNO055_I2C_bus_read(dev_addr, reg_addr, reg_data, cnt);
            }
        }

        u8 i = 0;
        for (u8 data : result) {
            reg_data[i++] = data;
        }

        return BNO055_SUCCESS;
    }

    // this is probably a communication failure, try again
    BNO055_reset_connection();
    return BNO055_I2C_bus_read(dev_addr, reg_addr, reg_data, cnt);
}
/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BNO055_delay_msek(u32 msek)
{
	//usleep(msek);
}

void BNO055_reset_connection()
{
    connection->flush();
    //usleep(200000);
    //connection->flush();
}