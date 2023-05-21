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
	void _positionCtrl();
	void _torqueCtrl();

	Vec34 _initFeetPos, _feetPos;
	Vec3 _initPos, _posGoal;
	Vec34 _posGoal4, _velGoal4, _forceGoal4;
	std::vector<Vec34> _posGoal4Vec, _velGoal4Vec, _forceGoal4Vec;
	Vec4 _eeContact;
	std::vector<Vec4> _eeContactVec;
	Vec3 _basePos;
	std::vector<Vec3> _basePosVec;
	Vec12 _targetPos;
	float _xMin, _xMax;
	float _yMin, _yMax;
	float _zMin, _zMax;
	Mat3 _Kp, _Kd;
	int _ctrl_index = 0;
	std::chrono::steady_clock::time_point last_time;
	std::chrono::steady_clock::time_point current_time;

};


#endif //TOWR_H
