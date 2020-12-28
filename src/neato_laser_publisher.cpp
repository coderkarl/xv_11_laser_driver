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

#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>
#include <xv_11_laser_driver/xv11_laser.h>
#include <std_msgs/msg/u_int16.hpp>

class LaserNode : public rclcpp::Node
{
public:
    LaserNode() : Node("xv11_laser")
    {
        port = declare_parameter("port", "/dev/ttyACM0");
        baud_rate = declare_parameter("baud_rate", 115200);
        frame_id = declare_parameter("frame_id", "neato_laser");
        firmware_number = declare_parameter("firmware_version", 2);

        RCLCPP_INFO(this->get_logger(), "XV11 Laser Node (port = %s) Initialized", port.c_str());
    }

    void output(std::string str)
    {
    	RCLCPP_INFO(this->get_logger(),
    			"%s", str.c_str() );
    }

    void output_error(std::string ex_str)
    {
    	RCLCPP_ERROR(this->get_logger(),
    			"Error instantiating laser object. Are you sure you have the correct port and baud rate?"
    			"Error was %s", ex_str.c_str() );
    }

public:
    std::string port;
    int baud_rate;
    std::string frame_id;
    int firmware_number;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LaserNode>();

  std_msgs::msg::UInt16 rpms;
  boost::asio::io_service io;

  try {
    xv_11_laser_driver::XV11Laser laser(node->port, node->baud_rate, node->firmware_number, io);
    auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", 1000);
    auto motor_pub = node->create_publisher<std_msgs::msg::UInt16>("rpms", 1000);

    while (rclcpp::ok()) {
      sensor_msgs::msg::LaserScan::SharedPtr scan(new sensor_msgs::msg::LaserScan);
      scan->header.frame_id = node->frame_id;
      scan->header.stamp = node->now();
      laser.poll(scan);
      rpms.data=laser.rpms;
      laser_pub->publish(*scan);
      motor_pub->publish(rpms);

    }
    laser.close();
    return 0;
  } catch (boost::system::system_error& ex) {
    node->output_error(ex.what());
    return -1;
  }
}
