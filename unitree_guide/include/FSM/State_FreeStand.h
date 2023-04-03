/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FREESTAND_H
#define FREESTAND_H

#include "FSM/FSMState.h"

class State_FreeStand : public FSMState
{
public:
	State_FreeStand(CtrlComponents *ctrlComp);
	~State_FreeStand() {}
	void enter();
	void run();
	void exit();
	FSMStateName checkChange();
private:

	Vec3 _initVecOX; // P_b0
	Vec34 _initVecXP; // P_si
	float _rowMax, _rowMin;
	float _pitchMax, _pitchMin;
	float _yawMax, _yawMin;
	float _heightMax, _heightMin;

	/*
	 * 根据目标欧拉角以及目标机身高度计算得到式6.3中的机身目标位姿T_sb，即变量Tsb，以及它的逆矩阵Tbs。
	 */
	Vec34 _calcOP(float row, float pitch, float yaw, float height);
	/*
	 * 根据足端目标位置求得各个关节的目标角度
	 */
	void _calcCmd(Vec34 vecOP);
};

#endif  // FREESTAND_H