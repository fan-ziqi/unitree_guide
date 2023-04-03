/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef IOMI_H
#define IOMI_H

#include "interface/IOInterface.h"
#include <CustomInterface.h>

#ifdef COMPILE_WITH_MOVE_BASE
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#endif  // COMPILE_WITH_MOVE_BASE

using CyberdogData = Robot_Data;
using CyberdogCmd = Motor_Cmd;

class IOMI : public IOInterface
{
public:
	IOMI();
	~IOMI() {}
	void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);

private:
	CyberdogData cyberdogData;
	CyberdogCmd cyberdogCmd;

#ifdef COMPILE_WITH_MOVE_BASE
	ros::NodeHandle _nh;
	ros::Publisher _pub;
	sensor_msgs::JointState _joint_state;
#endif  // COMPILE_WITH_MOVE_BASE
};

#endif  // IOMI_H