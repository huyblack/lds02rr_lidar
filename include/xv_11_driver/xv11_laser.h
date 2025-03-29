/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Eric Perko, Chad Rockey
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Case Western Reserve University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "sensor_msgs/msg/laser_scan.hpp"
#include <functional>
// #include <sensor_msgs/LaserScan.h>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <string>
#include "PID_v1_0_0.h" 

#define PACKET_LENGTH 22
#define OFFSET_TO_CRC_L (PACKET_LENGTH - 2)
#define OFFSET_TO_CRC_M (PACKET_LENGTH - 1)

namespace xv_11_driver
{
    class XV11Laser
    {
    public:
        uint16_t rpms; ///< @brief RPMS derived from the rpm bytes in an XV11 packet
        /**
         * @brief Constructs a new XV11Laser attached to the given serial port
         * @param port The string for the serial port device to attempt to connect to, e.g. "/dev/ttyUSB0"
         * @param baud_rate The baud rate to open the serial port at.
         * @param io Boost ASIO IO Service to use when creating the serial port object
         */
        XV11Laser(const std::string &port, uint32_t baud_rate, uint32_t firmware, boost::asio::io_service &io);

        /**
         * @brief Default destructor
         */
        ~XV11Laser() {};

        /**
         * @brief Set the motor speed in RPM.  This is a non-blocking call that will set the speed of the motor to the given value.
         * @param speed The speed in RPM to set the motor to.  This is a non-blocking call that will set the speed of the motor to the given value.
         */
        bool IsValidPacket(uint8_t *packet, int packetLength)
           
        /**
         * @brief Poll the laser to get a new scan. Blocks until a complete new scan is received or close is called.
         * @param scan LaserScan message pointer to fill in with the scan. The caller is responsible for filling in the ROS timestamp and frame_id
         */
        void poll(sensor_msgs::msg::LaserScan *scan);

        /**
         * @brief Close the driver down and prevent the polling loop from advancing
         */
        void close() { shutting_down_ = true; };

        // Các hàm điều khiển motor
        bool setScanRPM(float rpm);
        void setMotorPwmCallback(std::function<void(float)> cb);
        void enableMotor(bool enable);
        void updateMotorControl();
        bool motorCheck(); // kiểm tra xem RPM có nằm trong giới hạn không

    private:
        std::string port_;   ///< @brief The serial port the driver is attached to
        uint32_t baud_rate_; ///< @brief The baud rate for the serial connection
        uint32_t firmware_;  ///< @brief The firmware version to check.  Currently supports two different versions: 1 and 2.

        bool shutting_down_;              ///< @brief Flag for whether the driver is supposed to be shutting down or not
        boost::asio::serial_port serial_; ///< @brief Actual serial port object for reading/writing to the XV11 Laser Scanner

        // motor control variables
        bool motor_enabled = false;
        float scan_rpm_setpoint = 300.0; // ví dụ mặc định
        float measured_rpm = 0.0;
        float pwm_val = 0.5;
        float pwm_last = 0.5;
        unsigned long motor_check_timer = 0;
        unsigned long motor_check_interval = 200; // ms
        int scan_rpm_err = 0;
        const int scan_rpm_err_thresh = 10;
        
        // Callback để xuất pwm ra phần cứng, nếu được thiết lập
        std::function<void(float)> motor_callback;

        PID_v1 scanRpmPID;


        uint16_t motor_speed_; ///< @brief current motor speed as reported by the XV11.
    };
}
