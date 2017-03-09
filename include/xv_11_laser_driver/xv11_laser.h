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

#include <sensor_msgs/LaserScan.h>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <string>

namespace xv_11_laser_driver
{
    class XV11Laser
    {
    public:
        uint16_t rpms; //True RPM of LIDAR unit

        /**
         * Constructs a new XV11Laser attached to the given serial port
         * @param port      The string for the serial port name
         * @param baud_rate The baud rate for the serial port
         * @param io        The Boost ASIO Service to run for the serial port
         */
        XV11Laser(const std::string& port, const uint32_t baud_rate, boost::asio::io_service& io);

        ~XV11Laser() {};

        /**
         * Reads one packet of data
         * @param  scan  Scan to save packet into
         * @param  count Packet ID
         * @return       Lidar RPM
         */
        int read_Packet(sensor_msgs::LaserScan::Ptr scan, const uint8_t packetID);

        /**
        * Poll the laser to get a new scan. Blocks until a complete new scan is received or close is called.
        * @param scan LaserScan message pointer to fill in with the scan. The caller is responsible for filling in the
         *            ROS timestamp and frame_id
        */
        void poll(sensor_msgs::LaserScan::Ptr scan);

        /**
        * Close the driver down and prevent the polling loop from advancing
        */
        inline void close()
        { m_shutting_down = true; };

    private:
        std::string m_port; //Serial port the driver is connected to
        uint32_t m_baud_rate; //Baud rate for serial connection

        bool m_shutting_down; //Whether the driver should shut down
        boost::asio::serial_port m_serial; //Serial port object
        uint16_t m_motor_speed; //Motor RPM reported by the LIDAR unit

        uint8_t m_lastPacketID; //Initial state for packet ID

        /**
         * Filters RPM using a combination band pass filter
         * @param rpm Input RPM
         * @return    Filtered RPM
         */
        const int filterRPM(const int rpm) const;
    };
};
