#ifdef COMPILE_WITH_REAL_ROBOT

#if defined(ROBOT_TYPE_Mi)

#ifndef IOMI_H
#define IOMI_H

#include "interface/IOInterface.h"

// cyberdog接口
#include <CustomInterface.h>

#ifdef COMPILE_WITH_MOVE_BASE
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#endif  // COMPILE_WITH_MOVE_BASE

using CyberdogData = Robot_Data;
using CyberdogCmd = Motor_Cmd;

class IOMI : public CustomInterface, public IOInterface
{
public:
	IOMI();
	~IOMI() {}
	void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);

private:
	CyberdogData cyberdogData;
	CyberdogCmd cyberdogCmd;
	void UserCode() {}

#ifdef COMPILE_WITH_MOVE_BASE
	ros::NodeHandle _nh;
	ros::Publisher _pub;
	sensor_msgs::JointState _joint_state;
#endif  // COMPILE_WITH_MOVE_BASE
};

#endif  // IOMI_H

#endif

#endif  // COMPILE_WITH_REAL_ROBOT