#ifdef COMPILE_WITH_REAL_ROBOT
#if defined(ROBOT_TYPE_Mi)

#include "interface/IOMI.h"
#include "interface/KeyBoard.h"

#ifdef ROBOT_TYPE_Mi
IOMI::IOMI()
		: CustomInterface(500)
{
	std::cout << "The control interface for real robot (Cyberdog)" << std::endl;

	cmdPanel = new KeyBoard();

#ifdef COMPILE_WITH_MOVE_BASE
	_pub = _nh.advertise<sensor_msgs::JointState>("/realRobot/joint_states", 20);
	_joint_state.name.resize(12);
	_joint_state.position.resize(12);
	_joint_state.velocity.resize(12);
	_joint_state.effort.resize(12);
#endif  // COMPILE_WITH_MOVE_BASE
}
#endif

void IOMI::sendRecv(const LowlevelCmd *cmd, LowlevelState *state)
{
	for(int i(0); i < 12; ++i)
	{
//		_lowCmd.motorCmd[i].mode = cmd->motorCmd[i].mode;
		cyberdogCmd.q_des[i] = cmd->motorCmd[i].q;
		cyberdogCmd.qd_des[i] = cmd->motorCmd[i].dq;
		cyberdogCmd.kp_des[i] = cmd->motorCmd[i].Kp;
		cyberdogCmd.kd_des[i] = cmd->motorCmd[i].Kd;
		cyberdogCmd.tau_des[i] = cmd->motorCmd[i].tau;
	}

	for(int i(0); i < 12; ++i)
	{
		state->motorState[i].q = cyberdogData.q[i];
		state->motorState[i].dq = cyberdogData.qd[i];
//		state->motorState[i].ddq = _lowState.motorState[i].ddq;
		state->motorState[i].tauEst = cyberdogData.tau[i];
//		state->motorState[i].mode = _lowState.motorState[i].mode;
	}

	//IMU
	for(int i = 0; i < 3; i++)
	{
		state->imu.accelerometer[i] = cyberdogData.acc[i];
	}
	// 注意 Cyberdog SDK 的四元数顺序为 xyzw 需要转成 wxyz
	state->imu.quaternion[0] = cyberdogData.quat[1];
	state->imu.quaternion[1] = cyberdogData.quat[2];
	state->imu.quaternion[2] = cyberdogData.quat[3];
	state->imu.quaternion[3] = cyberdogData.quat[0];
	for(int i = 0; i < 3; i++)
	{
		state->imu.gyroscope[i] = cyberdogData.omega[i];
	}

//	cmdPanel->receiveHandle(&_lowState);
	state->userCmd = cmdPanel->getUserCmd();
	state->userValue = cmdPanel->getUserValue();

#ifdef COMPILE_WITH_MOVE_BASE
	_joint_state.header.stamp = ros::Time::now();
	_joint_state.name = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
						 "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
						 "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
						 "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
	for(int i(0); i<12; ++i){
		_joint_state.position[i] = state->motorState[i].q;
		_joint_state.velocity[i] = state->motorState[i].dq;
		_joint_state.effort[i]   = state->motorState[i].tauEst;
	}

	_pub.publish(_joint_state);
#endif  // COMPILE_WITH_MOVE_BASE
}

#endif
#endif  // COMPILE_WITH_REAL_ROBOT