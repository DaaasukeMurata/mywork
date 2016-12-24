#include <ros/ros.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt16MultiArray.h>
#include <sensor_msgs/Joy.h>
#include "teleop_rc.h"

TeleopRc::TeleopRc() : a_scale_(80), l_scale_(20)
{
    nh.param("scale_angular", a_scale_, a_scale_);
    nh.param("scale_linear", l_scale_, l_scale_);

    vel_pub_ = nh.advertise<std_msgs::UInt16MultiArray>("servo", 1);
    printf("TeleopRc constructor done\n");
}

void TeleopRc::move(TeleopRcDirection direction)
{
    uint16_t linear, angle;
    std_msgs::UInt16MultiArray cmd_msg;

    switch (direction)
    {
    case TELEOP_RC_FRONT:
	linear = 90 - l_scale_;
	angle = 90;
	break;
    case TELEOP_RC_RIGHT:
	linear = 90 - l_scale_;
	angle = 90 + a_scale_;
	break;
    case TELEOP_RC_LEFT:
	linear = 90 - l_scale_;
	angle = 90 - a_scale_;
	break;
    }

    cmd_msg.data.clear();
    cmd_msg.data.push_back((uint16_t)(angle));
    cmd_msg.data.push_back((uint16_t)(linear));
    vel_pub_.publish(cmd_msg);

    printf("linear:%d, angle:%d\n", linear, angle);
}

void TeleopRc::stop()
{
    uint16_t linear = 90;
    uint16_t angle = 90;
    std_msgs::UInt16MultiArray cmd_msg;

    cmd_msg.data.clear();
    cmd_msg.data.push_back((uint16_t)(angle));
    cmd_msg.data.push_back((uint16_t)(linear));
    vel_pub_.publish(cmd_msg);

    printf("linear:%d, angle:%d\n", linear, angle);
}
