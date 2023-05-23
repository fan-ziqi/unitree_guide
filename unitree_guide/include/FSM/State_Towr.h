#ifndef TOWR_H
#define TOWR_H

#include "FSM/FSMState.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include <xpp_msgs/RobotStateCartesian.h>

class State_Towr : public FSMState
{
public:
	State_Towr(CtrlComponents *ctrlComp);
	~State_Towr() {};
	void enter();
	void run();
	void exit();
	FSMStateName checkChange();
private:
	void _torqueCtrl();

	Vec34 _initFeetPos, _feetPos;
	Vec3 _initPos, _posGoal;

	Vec3 _basePosePos;
	std::vector<Vec3> _basePosePosVec;
	Vec4 _basePoseOri;
	std::vector<Vec4> _basePoseOriVec;
	Vec3 _baseTwiLin, _baseTwiAng, _baseAccLin, _baseAccAng;
	std::vector<Vec3> _baseTwiLinVec, _baseTwiAngVec, _baseAccLinVec, _baseAccAngVec;
	Vec34 _eePos4, _eeVel4, _eeAcc4, _eeForce4;
	std::vector<Vec34> _eePos4Vec, _eeVel4Vec, _eeVel4Acc, _eeForce4Vec;
	Vec4 _eeContact4;
	std::vector<Vec4> _eeContact4Vec;

	Mat3 _Kp, _Kd;
	int _ctrl_index = 0;
	std::chrono::steady_clock::time_point last_time;
	std::chrono::steady_clock::time_point current_time;
};


#endif //TOWR_H
