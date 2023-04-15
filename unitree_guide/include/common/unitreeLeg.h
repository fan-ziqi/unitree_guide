/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef UNITREELEG_H
#define UNITREELEG_H

#include "common/mathTypes.h"
#include "common/enumClass.h"

/*
 * 机器人三自由度机械腿的类QuadrupedLeg
 * 将机器人的腿抽象为一个类，并且把机器人运动学与静力学相关的计算抽象为这个类的成员函数
 */
class QuadrupedLeg
{
public:
	/*
	 * QuadrupedLeg类的构造函数，参数中legID为腿的编号，如图3.5所示，分别为0、1、2、3。
	 * 而abadLinkLength、hipLinkLength、kneeLinkLength则分别代表图5.3中的l_abad、l_hip、l_knee 三个长度。
	 * 最后的pHip2B代表从机身中心到该腿基座坐标系{0}的原点的向量。
	 */
	QuadrupedLeg(int legID, float abadLinkLength, float hipLinkLength,
	             float kneeLinkLength, Vec3 pHip2B);

	~QuadrupedLeg() {}

	/*
	 * 参照式5.11，计算当三个关节角度分别为q的时候，足端到基座坐标系{0} 原点的向量坐标。
	 * calc(计算)P(位置Position)Ee(末端执行器End-effector)2(到to)H(基座Home)
	 */
	Vec3 calcPEe2H(Vec3 q);

	/*
	 * 计算当三个关节角度分别为q的时候，足端到机身中心的向量坐标。
	 * calc(计算)P(位置Position)Ee(末端执行器End-effector)2(到to)B(机身Body)
	 */
	Vec3 calcPEe2B(Vec3 q);

	/*
	 * 参照式5.42，计算当关节角度为q，关节角速度为qd时，足端的速度向量。
	 */
	Vec3 calcVEe(Vec3 q, Vec3 qd);

	/*
	 * 参照第5.2节的逆向运动学，计算当足端坐标为pEe时腿上三个关节的角度
	 * frame表示坐标pEe所在的坐标系，可以为FrameType::HIP或FrameType::BODY，分别代表基座坐标系{0}和机身坐标系。
	 */
	Vec3 calcQ(Vec3 pEe, FrameType frame);

	/*
	 * 参照式5.43，根据当前腿上三个关节的角度和足端速度计算三个关节的角速度。
	 */
	Vec3 calcQd(Vec3 q, Vec3 vEe);

	/*
	 * 根据足端在frame坐标系下的位置坐标pEe和速度vEe计算三个关节的角速度
	 * 可以认为是融合了函数calcQd(q, vEe)和函数calcQ(pEe, frame)。
	 */
	Vec3 calcQd(Vec3 pEe, Vec3 vEe, FrameType frame);

	/*
	 * 参照式5.46，计算当该腿三个关节角度为q，足端对外作用力为force时，该腿三个关节的力矩。
	 */
	Vec3 calcTau(Vec3 q, Vec3 force);

	/*
	 * 参照式5.42，计算当三个关节角度分别为q时，该腿的雅可比矩阵。
	 * 同时，为了方便调用，我们将四足机器人A1的腿定义为QuadrupedLeg类的子类A1Leg，
	 * Go1机器人的腿也定义为QuadrupedLeg类的子类Go1Leg，可见A1Leg和Go1Leg的构造函数中保存了对应机器人腿长的默认参数。
	 */
	Mat3 calcJaco(Vec3 q);

	Vec3 getHip2B() { return _pHip2B; }
protected:
	float q1_ik(float py, float pz, float b2y);
	float q3_ik(float b3z, float b4z, float b);
	float q2_ik(float q1, float q3, float px,
	            float py, float pz, float b3z, float b4z);
	float _sideSign;
	const float _abadLinkLength, _hipLinkLength, _kneeLinkLength;

	// 从机身中心到该腿基座坐标系{0}的原点的向量
	const Vec3 _pHip2B;
};

class A1Leg : public QuadrupedLeg
{
public:
	A1Leg(const int legID, const Vec3 pHip2B)
			: QuadrupedLeg(legID, 0.0838, 0.2, 0.2, pHip2B) {}
	~A1Leg() {}
};

class Go1Leg : public QuadrupedLeg
{
public:
	Go1Leg(const int legID, const Vec3 pHip2B)
			: QuadrupedLeg(legID, 0.08, 0.213, 0.213, pHip2B) {}
	~Go1Leg() {}
};

class CYBERDOGLeg : public QuadrupedLeg
{
public:
	CYBERDOGLeg(const int legID, const Vec3 pHip2B)
			: QuadrupedLeg(legID, 0.10715, 0.2, 0.217, pHip2B) {}
	~CYBERDOGLeg() {}
};

#endif  // UNITREELEG_H