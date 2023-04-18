/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "control/Estimator.h"
#include "common/mathTools.h"
#include "common/enumClass.h"

Estimator::Estimator(QuadrupedRobot *robotModel, LowlevelState *lowState,
                     VecInt4 *contact, Vec4 *phase, double dt, Vec18 Qdig,
                     std::string testName)
		: _robModel(robotModel), _lowState(lowState), _contact(contact),
		  _phase(phase), _dt(dt), _Qdig(Qdig), _estName(testName)
{

	_initSystem();
}

Estimator::Estimator(QuadrupedRobot *robotModel, LowlevelState *lowState,
                     VecInt4 *contact, Vec4 *phase, double dt)
		: _robModel(robotModel), _lowState(lowState), _contact(contact),
		  _phase(phase), _dt(dt)
{

	for(int i(0); i < _Qdig.rows(); ++i)
	{
		if(i < 3)
		{
			_Qdig(i) = 0.0003;
		}
		else if(i < 6)
		{
			_Qdig(i) = 0.0003;
		}
		else
		{
			_Qdig(i) = 0.01;
		}
	}

	_estName = "current";

	_initSystem();

}

Estimator::~Estimator()
{
}

void Estimator::_initSystem()
{
	// 重力加速度
	_g << 0, 0, -9.81;
	_largeVariance = 100;

	_xhat.setZero();
	_u.setZero();

	// 系统矩阵A和输入矩阵B (式7.11)
	_A.setZero();
	_A.block(0, 0, 3, 3) = I3;
	_A.block(0, 3, 3, 3) = I3 * _dt;
	_A.block(3, 3, 3, 3) = I3;
	_A.block(6, 6, 12, 12) = I12;
	_B.setZero();
	_B.block(3, 0, 3, 3) = I3 * _dt;
	// 输出矩阵C (式7.14)
	_C.setZero();
	_C.block(0, 0, 3, 3) = -I3;
	_C.block(3, 0, 3, 3) = -I3;
	_C.block(6, 0, 3, 3) = -I3;
	_C.block(9, 0, 3, 3) = -I3;
	_C.block(12, 3, 3, 3) = -I3;
	_C.block(15, 3, 3, 3) = -I3;
	_C.block(18, 3, 3, 3) = -I3;
	_C.block(21, 3, 3, 3) = -I3;
	_C.block(0, 6, 12, 12) = I12;
	_C(24, 8) = 1;
	_C(25, 11) = 1;
	_C(26, 14) = 1;
	_C(27, 17) = 1;

	// 初始最优估计协方差设为一个很大的值
	_P.setIdentity();
	_P = _largeVariance * _P;

	// 将计算结果赋值给_RInit和_Cu, ONLY FOR CYBERDOG!!
	_RInit
			<< 0.836, 4.130, 1.295, -3.039, 2.592, 1.293, 3.347, -1.724, 1.346, -0.538, -3.370, 1.322, 0.081, 0.061, 0.366, 0.078, -0.010, 0.350, 0.094, -0.025, 0.304, 0.103, 0.025, 0.234, 0.000, 0.000, 0.000, 0.000,
			4.130, 25.179, 5.668, -19.479, 15.105, 5.669, 20.435, -10.470, 5.891, -3.215, -21.274, 5.792, 0.483, 0.486, 1.282, 0.524, 0.025, 1.253, 0.636, 0.031, 1.021, 0.727, 0.205, 0.685, 0.000, 0.000, 0.000, 0.000,
			1.295, 5.668, 8.325, -3.227, 3.446, 8.297, 5.079, -1.105, 8.609, 0.621, -3.211, 8.463, -0.170, -0.024, 0.678, -0.085, 0.056, 0.799, -0.019, -0.090, 0.809, -0.091, 0.054, 0.604, 0.000, 0.000, 0.000, 0.000,
			-3.039, -19.479, -3.227, 15.383, -11.623, -3.235, -15.726, 8.339, -3.356, 2.732, 16.805, -3.302, -0.342, -0.376, -0.544, -0.370, 0.020, -0.525, -0.451, -0.030, -0.366, -0.541, -0.123, -0.146, 0.000, 0.000, 0.000, 0.000,
			2.592, 15.105, 3.446, -11.623, 9.161, 3.447, 12.228, -6.362, 3.582, -2.015, -12.744, 3.522, 0.291, 0.286, 0.783, 0.310, 0.005, 0.760, 0.371, 0.003, 0.618, 0.425, 0.116, 0.415, 0.000, 0.000, 0.000, 0.000,
			1.293, 5.669, 8.297, -3.235, 3.447, 8.269, 5.077, -1.112, 8.580, 0.614, -3.220, 8.435, -0.165, -0.018, 0.671, -0.078, 0.061, 0.793, -0.013, -0.085, 0.802, -0.085, 0.058, 0.597, 0.000, 0.000, 0.000, 0.000,
			3.347, 20.435, 5.079, -15.726, 12.228, 5.077, 16.636, -8.363, 5.277, -2.466, -17.143, 5.187, 0.375, 0.373, 1.144, 0.413, 0.022, 1.124, 0.513, 0.014, 0.936, 0.576, 0.163, 0.648, 0.000, 0.000, 0.000, 0.000,
			-1.724, -10.470, -1.105, 8.339, -6.362, -1.112, -8.363, 4.731, -1.152, 1.729, 9.194, -1.135, -0.167, -0.183, -0.124, -0.172, 0.063, -0.099, -0.202, 0.018, -0.015, -0.262, -0.025, 0.089, 0.000, 0.000, 0.000, 0.000,
			1.346, 5.891, 8.609, -3.356, 3.582, 8.580, 5.277, -1.152, 8.903, 0.641, -3.343, 8.753, -0.161, -0.011, 0.730, -0.073, 0.070, 0.856, -0.003, -0.083, 0.867, -0.080, 0.069, 0.652, 0.000, 0.000, 0.000, 0.000,
			-0.538, -3.215, 0.621, 2.732, -2.015, 0.614, -2.466, 1.729, 0.641, 0.822, 3.077, 0.628, -0.057, -0.071, 0.208, -0.049, 0.046, 0.224, -0.047, 0.002, 0.243, -0.078, 0.010, 0.246, 0.000, 0.000, 0.000, 0.000,
			-3.370, -21.274, -3.211, 16.805, -12.744, -3.220, -17.143, 9.194, -3.343, 3.077, 18.404, -3.289, -0.383, -0.400, -0.649, -0.407, 0.041, -0.613, -0.490, -0.017, -0.431, -0.592, -0.118, -0.190, 0.000, 0.000, 0.000, 0.000,
			1.322, 5.792, 8.463, -3.302, 3.522, 8.435, 5.187, -1.135, 8.753, 0.628, -3.289, 8.605, -0.158, -0.008, 0.710, -0.070, 0.072, 0.835, -0.003, -0.078, 0.844, -0.078, 0.071, 0.638, 0.000, 0.000, 0.000, 0.000,
			0.081, 0.483, -0.170, -0.342, 0.291, -0.165, 0.375, -0.167, -0.161, -0.057, -0.383, -0.158, 8.913, 0.762, 3.735, 0.531, 0.461, 0.759, 0.568, 0.318, 0.692, 0.515, 0.396, 0.666, 0.000, 0.000, 0.000, 0.000,
			0.061, 0.486, -0.024, -0.376, 0.286, -0.018, 0.373, -0.183, -0.011, -0.071, -0.400, -0.008, 0.762, 8.066, -1.203, 0.461, 0.531, 0.615, 0.497, 0.487, 0.629, 0.420, 0.534, 0.718, 0.000, 0.000, 0.000, 0.000,
			0.366, 1.282, 0.678, -0.544, 0.783, 0.671, 1.144, -0.124, 0.730, 0.208, -0.649, 0.710, 3.735, -1.203, 5.202, 0.726, 0.518, 2.074, 0.767, 0.375, 1.909, 0.702, 0.519, 1.796, 0.000, 0.000, 0.000, 0.000,
			0.078, 0.524, -0.085, -0.370, 0.310, -0.078, 0.413, -0.172, -0.073, -0.049, -0.407, -0.070, 0.531, 0.461, 0.726, 8.874, 1.155, 1.878, 0.583, 0.350, 0.699, 0.595, 0.482, 0.668, 0.000, 0.000, 0.000, 0.000,
			-0.010, 0.025, 0.056, 0.020, 0.005, 0.061, 0.022, 0.063, 0.070, 0.046, 0.041, 0.072, 0.461, 0.531, 0.518, 1.155, 8.040, 3.829, 0.491, 0.502, 0.680, 0.413, 0.550, 0.758, 0.000, 0.000, 0.000, 0.000,
			0.350, 1.253, 0.799, -0.525, 0.760, 0.793, 1.124, -0.099, 0.856, 0.224, -0.613, 0.835, 0.759, 0.615, 2.074, 1.878, 3.829, 5.200, 0.812, 0.495, 1.984, 0.736, 0.653, 1.903, 0.000, 0.000, 0.000, 0.000,
			0.094, 0.636, -0.019, -0.451, 0.371, -0.013, 0.513, -0.202, -0.003, -0.047, -0.490, -0.003, 0.568, 0.497, 0.767, 0.583, 0.491, 0.812, 8.570, 1.044, 3.941, 0.548, 0.403, 0.654, 0.000, 0.000, 0.000, 0.000,
			-0.025, 0.031, -0.090, -0.030, 0.003, -0.085, 0.014, 0.018, -0.083, 0.002, -0.017, -0.078, 0.318, 0.487, 0.375, 0.350, 0.502, 0.495, 1.044, 7.062, -0.653, 0.402, 0.532, 0.687, 0.000, 0.000, 0.000, 0.000,
			0.304, 1.021, 0.809, -0.366, 0.618, 0.802, 0.936, -0.015, 0.867, 0.243, -0.431, 0.844, 0.692, 0.629, 1.909, 0.699, 0.680, 1.984, 3.941, -0.653, 5.391, 0.657, 0.679, 1.942, 0.000, 0.000, 0.000, 0.000,
			0.103, 0.727, -0.091, -0.541, 0.425, -0.085, 0.576, -0.262, -0.080, -0.078, -0.592, -0.078, 0.515, 0.420, 0.702, 0.595, 0.413, 0.736, 0.548, 0.402, 0.657, 7.972, 1.094, 1.997, 0.000, 0.000, 0.000, 0.000,
			0.025, 0.205, 0.054, -0.123, 0.116, 0.058, 0.163, -0.025, 0.069, 0.010, -0.118, 0.071, 0.396, 0.534, 0.519, 0.482, 0.550, 0.653, 0.403, 0.532, 0.679, 1.094, 6.943, 3.742, 0.000, 0.000, 0.000, 0.000,
			0.234, 0.685, 0.604, -0.146, 0.415, 0.597, 0.648, 0.089, 0.652, 0.246, -0.190, 0.638, 0.666, 0.718, 1.796, 0.668, 0.758, 1.903, 0.654, 0.687, 1.942, 1.997, 3.742, 5.328, 0.000, 0.000, 0.000, 0.000,
			0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 1.000, 0.000, 0.000, 0.000,
			0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 1.000, 0.000, 0.000,
			0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 1.000, 0.000,
			0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 1.000;

	_Cu << 206.473, -76.925, -22.224,
			-76.925, 198.944, 99.339,
			-22.224, 99.339, 277.110;

	_QInit = _Qdig.asDiagonal();
	_QInit += _B * _Cu * _B.transpose();

	// 对R矩阵和Cu矩阵的测量
	_RCheck = new AvgCov(28, _estName + " R");
	_uCheck = new AvgCov(3, _estName + " u");

	_vxFilter = new LPFilter(_dt, 3.0);
	_vyFilter = new LPFilter(_dt, 3.0);
	_vzFilter = new LPFilter(_dt, 3.0);


	/* ROS odometry publisher */
#ifdef COMPILE_WITH_MOVE_BASE
	_pub = _nh.advertise<nav_msgs::Odometry>("odom", 1);
#endif  // COMPILE_WITH_MOVE_BASE
}

void Estimator::run()
{
//	// 测量R和Cu，需要在控制程序开始的10s之内切换到FixedStand模式
//	_RCheck->measure(_y);
//	_uCheck->measure(_u);

	_feetH.setZero();
	_feetPosGlobalKine = _robModel->getFeet2BPositions(*_lowState, FrameType::GLOBAL);
	_feetVelGlobalKine = _robModel->getFeet2BVelocities(*_lowState, FrameType::GLOBAL);

	_Q = _QInit;
	_R = _RInit;

	for(int i(0); i < 4; ++i)
	{
		if((*_contact)(i) == 0)
		{
			_Q.block(6 + 3 * i, 6 + 3 * i, 3, 3) = _largeVariance * I3;
			_R.block(12 + 3 * i, 12 + 3 * i, 3, 3) = _largeVariance * I3;
			_R(24 + i, 24 + i) = _largeVariance;
		}
		else
		{
			_trust = windowFunc((*_phase)(i), 0.2);
			_Q.block(6 + 3 * i, 6 + 3 * i, 3, 3) = (1 + (1 - _trust) * _largeVariance) * _QInit.block(6 + 3 * i, 6 + 3 * i, 3, 3);
			_R.block(12 + 3 * i, 12 + 3 * i, 3, 3) = (1 + (1 - _trust) * _largeVariance) * _RInit.block(12 + 3 * i, 12 + 3 * i, 3, 3);
			_R(24 + i, 24 + i) = (1 + (1 - _trust) * _largeVariance) * _RInit(24 + i, 24 + i);
		}
		_feetPos2Body.segment(3 * i, 3) = _feetPosGlobalKine.col(i);
		_feetVel2Body.segment(3 * i, 3) = _feetVelGlobalKine.col(i);
	}

	_rotMatB2G = _lowState->getRotMat();
	_u = _rotMatB2G * _lowState->getAcc() + _g;
	_xhat = _A * _xhat + _B * _u;
	_yhat = _C * _xhat;
	_y << _feetPos2Body, _feetVel2Body, _feetH;

	_Ppriori = _A * _P * _A.transpose() + _Q;
	_S = _R + _C * _Ppriori * _C.transpose();
	_Slu = _S.lu();
	_Sy = _Slu.solve(_y - _yhat);
	_Sc = _Slu.solve(_C);
	_SR = _Slu.solve(_R);
	_STC = (_S.transpose()).lu().solve(_C);
	_IKC = I18 - _Ppriori * _C.transpose() * _Sc;

	_xhat += _Ppriori * _C.transpose() * _Sy;
	_P = _IKC * _Ppriori * _IKC.transpose()
	     + _Ppriori * _C.transpose() * _SR * _STC * _Ppriori.transpose();

	_vxFilter->addValue(_xhat(3));
	_vyFilter->addValue(_xhat(4));
	_vzFilter->addValue(_xhat(5));

#ifdef COMPILE_WITH_MOVE_BASE
	if(_count % ((int)( 1.0/(_dt*_pubFreq))) == 0){
		_currentTime = ros::Time::now();
		/* tf */
		_odomTF.header.stamp = _currentTime;
		_odomTF.header.frame_id = "odom";
		_odomTF.child_frame_id  = "base";

		_odomTF.transform.translation.x = _xhat(0);
		_odomTF.transform.translation.y = _xhat(1);
		_odomTF.transform.translation.z = _xhat(2);
		_odomTF.transform.rotation.w = _lowState->imu.quaternion[0];
		_odomTF.transform.rotation.x = _lowState->imu.quaternion[1];
		_odomTF.transform.rotation.y = _lowState->imu.quaternion[2];
		_odomTF.transform.rotation.z = _lowState->imu.quaternion[3];

		_odomBroadcaster.sendTransform(_odomTF);

		/* odometry */
		_odomMsg.header.stamp = _currentTime;
		_odomMsg.header.frame_id = "odom";

		_odomMsg.pose.pose.position.x = _xhat(0);
		_odomMsg.pose.pose.position.y = _xhat(1);
		_odomMsg.pose.pose.position.z = _xhat(2);

		_odomMsg.pose.pose.orientation.w = _lowState->imu.quaternion[0];
		_odomMsg.pose.pose.orientation.x = _lowState->imu.quaternion[1];
		_odomMsg.pose.pose.orientation.y = _lowState->imu.quaternion[2];
		_odomMsg.pose.pose.orientation.z = _lowState->imu.quaternion[3];
		_odomMsg.pose.covariance = _odom_pose_covariance;

		_odomMsg.child_frame_id = "base";
		_velBody = _rotMatB2G.transpose() * _xhat.segment(3, 3);
		_wBody   = _lowState->imu.getGyro();
		_odomMsg.twist.twist.linear.x = _velBody(0);
		_odomMsg.twist.twist.linear.y = _velBody(1);
		_odomMsg.twist.twist.linear.z = _velBody(2);
		_odomMsg.twist.twist.angular.x = _wBody(0);
		_odomMsg.twist.twist.angular.y = _wBody(1);
		_odomMsg.twist.twist.angular.z = _wBody(2);
		_odomMsg.twist.covariance = _odom_twist_covariance;

		_pub.publish(_odomMsg);
		_count = 1;
	}
	++_count;
#endif  // COMPILE_WITH_MOVE_BASE
}

Vec3 Estimator::getPosition()
{
	return _xhat.segment(0, 3);
}

Vec3 Estimator::getVelocity()
{
	return _xhat.segment(3, 3);
}

Vec3 Estimator::getFootPos(int i)
{
	return getPosition() + _lowState->getRotMat() * _robModel->getFootPosition(*_lowState, i, FrameType::BODY);
}

Vec34 Estimator::getFeetPos()
{
	Vec34 feetPos;
	for(int i(0); i < 4; ++i)
	{
		feetPos.col(i) = getFootPos(i);
	}
	return feetPos;
}

Vec34 Estimator::getFeetVel()
{
	Vec34 feetVel = _robModel->getFeet2BVelocities(*_lowState, FrameType::GLOBAL);
	for(int i(0); i < 4; ++i)
	{
		feetVel.col(i) += getVelocity();
	}
	return feetVel;
}

Vec34 Estimator::getPosFeet2BGlobal()
{
	Vec34 feet2BPos;
	for(int i(0); i < 4; ++i)
	{
		feet2BPos.col(i) = getFootPos(i) - getPosition();
	}
	return feet2BPos;
}

