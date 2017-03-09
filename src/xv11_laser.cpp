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

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <functional>
#include <iterator>

#include "xv_11_laser_driver/xv11_laser.h"

namespace xv_11_laser_driver
{
    XV11Laser::XV11Laser(const std::string &port, const uint32_t baud_rate, boost::asio::io_service &io) :
            m_port(port),
            m_baud_rate(baud_rate),
            m_shutting_down(false),
            m_serial(io, m_port)
    {
        m_serial.set_option(boost::asio::serial_port_base::baud_rate(m_baud_rate));
        m_lastPacketID = 0; // set ID to impossible value to trigger first scan behavior
    }

    int XV11Laser::read_Packet(sensor_msgs::LaserScan::Ptr scan, const uint8_t packetID)
    {
        boost::array<uint8_t, 20> raw_bytes;
        boost::asio::read(m_serial, boost::asio::buffer(&raw_bytes[0], 20));

        uint8_t dataStartIndex;
        uint16_t firstIndex = 4 * (packetID - 160), range, intensity;

        for (int i = 0; i < 4; i++)
        {
            dataStartIndex = 2 + 4 * i;

            if (raw_bytes[dataStartIndex + 1] & 0x80)
                continue; // Skip to next point

            range = (((uint16_t)(0x3f & raw_bytes[dataStartIndex + 1])) << 8) + raw_bytes[dataStartIndex];
            intensity = (((uint16_t) raw_bytes[dataStartIndex + 3]) << 8) + raw_bytes[dataStartIndex + 2];
            scan->ranges[firstIndex + i] = range / 1000.0;
            scan->intensities[firstIndex + i] = intensity;
        }

        return (((uint16_t) raw_bytes[1]) << 8) + raw_bytes[0];
    }

    void XV11Laser::poll(sensor_msgs::LaserScan::Ptr scan)
    {
        boost::array<uint8_t, 1> byte;
        uint8_t good_packets = 0;
        uint32_t sum_motor_speed = 0;
        double average_RPM;

        rpms = 0;
        constexpr int offset = 13;
        scan->angle_min = 0.0 + (offset / 180.0) * M_PI;
        scan->angle_max = 2.0 * M_PI + (offset / 180.0) * M_PI;
        scan->angle_increment = (2.0 * M_PI / 360.0);
        scan->range_min = 0.06;
        scan->range_max = 5.0;
        scan->ranges.resize(360);
        scan->intensities.resize(360);

        //every scan except first scan use the saved packetID for the first packet of each scan
        if (m_lastPacketID != 0)
        {
            sum_motor_speed += filterRPM(read_Packet(scan, m_lastPacketID));
            good_packets++;
        }

        while (true)
        {
            do
            {
                boost::asio::read(m_serial, boost::asio::buffer(&byte[0], 1));
            } while (byte[0] != 0xFA);

            boost::asio::read(m_serial, boost::asio::buffer(&byte[0], 1));

            if (byte[0] < 160 || byte[0] > 249) //160 = 0xA0, 249 = 0xF9
                continue;

            //read start and count byte
            if (byte[0] < m_lastPacketID) //if new scan was detected
                break;

            m_lastPacketID = byte[0];
            sum_motor_speed += filterRPM(read_Packet(scan, m_lastPacketID));
            good_packets++;
        }

        //Assume average rpm is 250 if we don't have any data
        average_RPM = good_packets != 0 ? (1.0 * sum_motor_speed) / (good_packets * 64.0) : 250;
        rpms = average_RPM;
        scan->time_increment = 1 / (6 * average_RPM);
        m_lastPacketID = byte[0]; //save packetID for next scan
    }

    const int XV11Laser::filterRPM(const int rpm) const
    {
        constexpr int len = 51; //sample count
        constexpr int medianElement = (int) ((len + 0.5) / 2.0);
        static int count = -1, lastRPM = 250;
        static int rpms[len];

        rpms[count++ % len] = rpm;

        if (count > len)
        {
            std::vector<int> rpmSorted(std::begin(rpms), std::end(rpms));
            std::sort(rpmSorted.begin(), rpmSorted.end(), std::greater<int>());
            return lastRPM = rpmSorted[medianElement];
        }

        return lastRPM;
    }
};
