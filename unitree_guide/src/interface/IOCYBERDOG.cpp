#ifdef COMPILE_WITH_REAL_ROBOT

#if defined(ROBOT_TYPE_CYBERDOG)

#include "interface/IOCYBERDOG.h"
#include "interface/KeyBoard.h"

//#define DEBUG_MOTOR

IOCYBERDOG::IOCYBERDOG()
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

void IOCYBERDOG::sendRecv(const LowlevelCmd *cmd, LowlevelState *state)
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
	// Cyberdog SDK 的四元数顺序为 wxyz
	for(int i = 0; i < 4; i++)
	{
		state->imu.quaternion[i] = cyberdogData.quat[i];
	}
	for(int i = 0; i < 3; i++)
	{
		state->imu.gyroscope[i] = cyberdogData.omega[i];
	}

//	cmdPanel->receiveHandle(&_lowState);
	state->userCmd = cmdPanel->getUserCmd();
	state->userValue = cmdPanel->getUserValue();

#ifdef COMPILE_WITH_MOVE_BASE
	_joint_state.header.stamp = ros::Time::now();
	// 需要修改
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

void IOCYBERDOG::UserCode()
{
	cyberdogData = robot_data;
	motor_cmd = cyberdogCmd;

#ifdef DEBUG_MOTOR

	if((count++) % 1000 == 0)
	{
		printf("interval:---------%.4f-------------\n", cyberdogData.ctrl_topic_interval);
		printf("rpy [3]:");
		for(int i = 0; i < 3; i++)
			printf(" %.2f", cyberdogData.rpy[i]);
		printf("\nacc [3]:");
		for(int i = 0; i < 3; i++)
			printf(" %.2f", cyberdogData.acc[i]);
		printf("\nquat[4]:");
		for(int i = 0; i < 4; i++)
			printf(" %.2f", cyberdogData.quat[i]);
		printf("\nomeg[3]:");
		for(int i = 0; i < 3; i++)
			printf(" %.2f", cyberdogData.omega[i]);
		printf("\nq  [12]:");
		for(int i = 0; i < 12; i++)
			printf(" %.2f", cyberdogData.q[i]);
		printf("\nqd [12]:");
		for(int i = 0; i < 12; i++)
			printf(" %.2f", cyberdogData.qd[i]);
		printf("\ntau[12]:");
		for(int i = 0; i < 12; i++)
			printf(" %.2f", cyberdogData.tau[i]);
		printf("\nq_des[12]:");
		for(int i = 0; i < 12; i++)
			printf(" %.2f", cyberdogCmd.q_des[i]);
		printf("\nqd_des[12]:");
		for(int i = 0; i < 12; i++)
			printf(" %.2f", cyberdogCmd.qd_des[i]);
		printf("\nkp_des[12]:");
		for(int i = 0; i < 12; i++)
			printf(" %.2f", cyberdogCmd.kp_des[i]);
		printf("\nkd_des[12]:");
		for(int i = 0; i < 12; i++)
			printf(" %.2f", cyberdogCmd.kd_des[i]);
		printf("\ntau_des[12]:");
		for(int i = 0; i < 12; i++)
			printf(" %.2f", cyberdogCmd.tau_des[i]);
		printf("\n\n");
	}
#endif // DEBUG_MOTOR
}

#endif

#endif  // COMPILE_WITH_REAL_ROBOT