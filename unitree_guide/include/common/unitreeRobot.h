/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef UNITREEROBOT_H
#define UNITREEROBOT_H

#include "common/unitreeLeg.h"
#include "message/LowlevelState.h"

/*
 * 整机运动学
 */
class QuadrupedRobot
{
public:
	QuadrupedRobot() {};
	~QuadrupedRobot() {}

	/*
	 * 式6.2，计算p_b0(0)，即0号足端在body坐标系下的初始坐标
	 */
	Vec3 getX(LowlevelState &state);

	/*
	 * 式6.1，计算p_si，分别对应四列，即初始状态下足端在世界坐标系s下的坐标
	 */
	Vec34 getVecXP(LowlevelState &state);

	/*
	 * 逆向运动学计算，获取当前机器人全部12 个关节的角度。
	 * 其中vecP的每一列分别代表四个足端的位置坐标，frame代表vecP中位置坐标所在的坐标系，
	 * frame只可以是FrameType::HIP 或FrameType::BODY。
	 */
	Vec12 getQ(const Vec34 &feetPosition, FrameType frame);

	/*
	 * 逆向微分运动学计算，获取当前机器人全部12个关节的角速度。
	 * 其中pos代表四个足端的位置，vel代表四个足端的速度，frame和函数getQ相同。
	 */
	Vec12 getQd(const Vec34 &feetPosition, const Vec34 &feetVelocity, FrameType frame);

	/*
	 * 机器人静力学计算，获取当前机器人全部12个关节的力矩。
	 * 其中q代表当前12个关节的角度，feetForce为四个足端的对外作用力。
	 */
	Vec12 getTau(const Vec12 &q, const Vec34 feetForce);

	/*
	 * 正向运动学计算，获取机器人第id条腿在frame坐标系下的位置坐标。
	 * state是一个LowlevelState类型的结构体，其中包含了机器人所有关节角度信息。
	 * frame只可以是FrameType::HIP 或FrameType::BODY。
	 */
	Vec3 getFootPosition(LowlevelState &state, int id, FrameType frame);

	/*
	 * 正向微分运动学计算，获取机器人第id条腿的速度向量。
	 */
	Vec3 getFootVelocity(LowlevelState &state, int id);

	/*
	 * 获取所有足端相对于机身中心的位置坐标，
	 * 参数frame除了可以是FrameType::HIP 和FrameType::BODY外，还可以是代表世界坐标系的FrameType::GLOBAL。
	 */
	Vec34 getFeet2BPositions(LowlevelState &state, FrameType frame);

	/*
	 * 获取所有足端相对于机身中心的速度向量，当frame = FrameType::GLOBAL时，参照式6.13。
	 */
	Vec34 getFeet2BVelocities(LowlevelState &state, FrameType frame);


	/*
	 * 获取第legID条腿在state状态下的雅可比矩阵。
	 * 不同机器人的参数各有不同，所以我们为QuadrupedRobot类创建了两个子类，即A1机器人的A1Robot 和Go1 机器人的Go1Robot。
	 * 在这两个子类中对一些变量进行了初始化，可以在A1Robot和Go1Robot的构造函数中查看：
	 */
	Mat3 getJaco(LowlevelState &state, int legID);
	Vec2 getRobVelLimitX() { return _robVelLimitX; }
	Vec2 getRobVelLimitY() { return _robVelLimitY; }
	Vec2 getRobVelLimitYaw() { return _robVelLimitYaw; }
	Vec34 getFeetPosIdeal() { return _feetPosNormalStand; }
	double getRobMass() { return _mass; }
	Vec3 getPcb() { return _pcb; }
	Mat3 getRobInertial() { return _Ib; }

protected:

	QuadrupedLeg *_Legs[4]; // 四个QuadrupedLeg实例，每个元素都是一个QuadrupedLeg类的指针，分别代表四条腿。
	Vec2 _robVelLimitX; // 机器人在机身坐标系{b}下x轴方向的平移速度区间。
	Vec2 _robVelLimitY; // 机器人在机身坐标系{b}下y轴方向的平移速度区间。
	Vec2 _robVelLimitYaw; // 机器人在机身坐标系{b}下绕z轴方向的转动角速度区间。
	Vec34 _feetPosNormalStand; // 代表各个足端中性落脚点在机身坐标系{b}下的坐标，将在第9.2节中使用到
	double _mass; // 机器人简化模型的质量。
	Vec3 _pcb;
	Mat3 _Ib; // 机器人简化模型在机身坐标系{b}下的转动惯量。
};

class A1Robot : public QuadrupedRobot
{
public:
	A1Robot();
	~A1Robot() {}
};

class Go1Robot : public QuadrupedRobot
{
public:
	Go1Robot();
	~Go1Robot() {};
};

class CYBERDOGRobot : public QuadrupedRobot
{
public:
	CYBERDOGRobot();
	~CYBERDOGRobot() {};
};

#endif  // UNITREEROBOT_H