#include "FSM/State_Towr.h"

State_Towr::State_Towr(CtrlComponents *ctrlComp)
		: FSMState(ctrlComp, FSMStateName::TOWR, "Towr")
{
	// 开启安全检测
	this->checkSafeOrientation = false;
}

void State_Towr::enter()
{
	for(int i = 0; i < 4; i++)
	{
		if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO)
		{
			_lowCmd->setSimStanceGain(i);
		}
		else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT)
		{
			_lowCmd->setRealStanceGain(i);
		}
		_lowCmd->setZeroDq(i);
		_lowCmd->setZeroTau(i);
	}
//	_lowCmd->setSwingGain(0);
//	_lowCmd->setSwingGain(1);
//	_lowCmd->setSwingGain(2);
//	_lowCmd->setSwingGain(3);

	_Kp = Vec3(20, 20, 50).asDiagonal();
	_Kd = Vec3(5, 5, 20).asDiagonal();

	for(int i = 0; i < 12; i++)
	{
		_lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
	}

	_initFeetPos = _ctrlComp->robotModel->getFeet2BPositions(*_lowState, FrameType::HIP);
	_feetPos = _initFeetPos;
	_initPos = _initFeetPos.col(1);

//	_ctrlComp->setAllSwing();

	// TOWR START

	std::string src_bag = "/mnt/f/我的文件/研究生/四足项目/宇树/unitree_guide/src/thirdparty/xpp_msgs/bags/cyberdog_bag_15161.bag";
	std::string state_topic = "/xpp/state_des";

	rosbag::Bag bag;
	bag.open(src_bag, rosbag::bagmode::Read);
	std::vector<std::string> topics;
	topics.push_back(std::string(state_topic));
	rosbag::View view(bag, rosbag::TopicQuery(topics));
	// rosbag::View view(bag);
	int i = 0;
	int ee_index[4] = {1, 0, 3, 2};
	for(auto m: view)
	{
		xpp_msgs::RobotStateCartesian::ConstPtr state = m.instantiate<xpp_msgs::RobotStateCartesian>();
		if(state == nullptr)
		{
			std::cout << "null " << std::endl;
		}
		else
		{
//			std::cout << "time_from_start:\t" << state->time_from_start << std::endl;
//			std::cout << "base:\t" << std::endl;
//			std::cout << "\tpose:\t" << std::endl;
//			std::cout << "\t\tposition:\t"
//			          << state->base.pose.position.x << "\t"
//			          << state->base.pose.position.y << "\t"
//			          << state->base.pose.position.z << std::endl;
			_basePos << state->base.pose.position.x, state->base.pose.position.y, state->base.pose.position.z;
			_basePosVec.push_back(_basePos);
//			std::cout << "\t\torientation:\t"
//			          << state->base.pose.orientation.x << "\t"
//			          << state->base.pose.orientation.y << "\t"
//			          << state->base.pose.orientation.z << "\t"
//			          << state->base.pose.orientation.w << std::endl;
//
//			std::cout << "\ttwist:\t" << std::endl;
//			std::cout << "\t\tlinear:\t"
//			          << state->base.twist.linear.x << "\t"
//			          << state->base.twist.linear.y << "\t"
//			          << state->base.twist.linear.z << std::endl;
//			std::cout << "\t\tangular:\t"
//			          << state->base.twist.angular.x << "\t"
//			          << state->base.twist.angular.y << "\t"
//			          << state->base.twist.angular.z << std::endl;
//
//			std::cout << "\taccel:\t" << std::endl;
//			std::cout << "\t\tlinear:\t"
//			          << state->base.accel.linear.x << "\t"
//			          << state->base.accel.linear.y << "\t"
//			          << state->base.accel.linear.z << std::endl;
//			std::cout << "\t\tangular:\t"
//			          << state->base.accel.angular.x << "\t"
//			          << state->base.accel.angular.y << "\t"
//			          << state->base.accel.angular.z << std::endl;
//
//			std::cout << "\tee_motion:\t" << std::endl;
			i = 0;
			for(auto &ee: state->ee_motion)
			{
//				std::cout << "\t\tpos:\t"
//				          << ee.pos.x << "\t"
//				          << ee.pos.y << "\t"
//				          << ee.pos.z << std::endl;
				_posGoal4.col(ee_index[i]) << ee.pos.x, ee.pos.y, ee.pos.z;

//				std::cout << "\t\tvel:\t"
//				          << ee.vel.x << "\t"
//				          << ee.vel.y << "\t"
//				          << ee.vel.z << std::endl;
				_velGoal4.col(ee_index[i]) << ee.vel.x, ee.vel.y, ee.vel.z;
//				std::cout << "\t\tacc:\t"
//				          << ee.acc.x << "\t"
//				          << ee.acc.y << "\t"
//				          << ee.acc.z << std::endl;
				i++;
			}
			_posGoal4Vec.push_back(_posGoal4);
			_velGoal4Vec.push_back(_velGoal4);

//			std::cout << "\tee_forces:\t" << std::endl;
			i = 0;
			for(auto &ee_force: state->ee_forces)
			{
//				std::cout << "\t\t"
//				          << ee_force.x << "\t"
//				          << ee_force.y << "\t"
//				          << ee_force.z << std::endl;
				_forceGoal4.col(ee_index[i]) << ee_force.x, ee_force.y, ee_force.z;
				i++;
			}
			_forceGoal4Vec.push_back(_forceGoal4);

//			std::cout << "\tee_contact:\t" << std::endl;
			i = 0;
			for(auto &contact: state->ee_contact)
			{
				if(contact == true)
				{
					_eeContact(ee_index[i]) = 1;
//					std::cout << "1\t" << contact;
				}
				else
				{
					_eeContact(ee_index[i]) = 0;
//					std::cout << "0\t" << contact;
				}
				i++;
			}
			_eeContactVec.push_back(_eeContact);
//			std::cout << std::endl << std::endl;

		}
	}
	bag.close();

//	for(const auto &posGoal4: _posGoal4Vec)
//	{
//		for(int i = 0; i < 3; ++i)
//		{
//			for(int j = 0; j < 4; ++j)
//			{
//				std::cout << posGoal4(i, j) << " ";
//			}
//			std::cout << std::endl;
//		}
//		std::cout << std::endl;
//	}
//	for(const auto &torGoal4: _forceGoal4Vec)
//	{
//		for(int i = 0; i < 3; ++i)
//		{
//			for(int j = 0; j < 4; ++j)
//			{
//				std::cout << torGoal4(i, j) << " ";
//			}
//			std::cout << std::endl;
//		}
//		std::cout << std::endl;
//	}
//	for(const auto &basePos: _basePosVec)
//	{
//		for(int i = 0; i < 3; ++i)
//		{
//			std::cout << basePos(i) << " ";
//		}
//		std::cout << std::endl;
//	}
//	for(const auto &eeContact: _eeContactVec)
//	{
//		for(int i = 0; i < 3; ++i)
//		{
//			std::cout << eeContact(i) << " ";
//		}
//		std::cout << std::endl;
//	}
	_ctrlComp->ioInter->zeroCmdPanel();
	_ctrl_index = 0;
	last_time = std::chrono::steady_clock::now();
}

void State_Towr::run()
{
	int update_interval = 10; // 更新间隔，单位为毫秒

	_forceGoal4 = _forceGoal4Vec.at(_ctrl_index);
	_posGoal4 = _posGoal4Vec.at(_ctrl_index);
	_velGoal4 = _velGoal4Vec.at(_ctrl_index);
	_eeContact = _eeContactVec.at(_ctrl_index);

	for(int i = 0; i < 4; ++i)
	{
		std::cout << _eeContact(i) << " ";
		if(_eeContact(i) == 0)
		{
			_lowCmd->setSwingGain(i);
		}
		else
		{
			_lowCmd->setStableGain(i);
		}
	}
	std::cout << std::endl;

//	_positionCtrl();
	_torqueCtrl();

	current_time = std::chrono::steady_clock::now();
	auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count();


	if(elapsed_ms >= update_interval)
	{
		if(_ctrl_index < _forceGoal4Vec.size() - 1)
		{
			std::cout << _ctrl_index << std::endl;
			_ctrl_index++;
		}
		last_time = std::chrono::steady_clock::now();
	}

}

void State_Towr::exit()
{
	_forceGoal4Vec.clear();
	_posGoal4Vec.clear();
	_eeContactVec.clear();
	_ctrlComp->ioInter->zeroCmdPanel();
}

FSMStateName State_Towr::checkChange()
{
	if(_lowState->userCmd == UserCommand::L2_B)
	{
		return FSMStateName::PASSIVE;
	}
	else if(_lowState->userCmd == UserCommand::L2_A)
	{
		return FSMStateName::FIXEDSTAND;
	}
	else
	{
		return FSMStateName::TOWR;
	}
}

void State_Towr::_positionCtrl()
{
	_feetPos.col(0) = _posGoal;
	_targetPos = _ctrlComp->robotModel->getQ(_feetPos, FrameType::HIP);
	_lowCmd->setQ(_targetPos);
}

void State_Towr::_torqueCtrl()
{
	Vec3 pos, vel;
	Mat3 jaco;
	Vec12 _tau_des, _tau_pd, _tau;

	for(int i = 0; i < 4; i++)
	{
		pos = _ctrlComp->robotModel->getFootPosition(*_lowState, i, FrameType::HIP);
		vel = _ctrlComp->robotModel->getFootVelocity(*_lowState, i);
		jaco = _ctrlComp->robotModel->getJaco(*_lowState, i);

		_tau_des.segment(i * 3, 3) = jaco.transpose() * -_forceGoal4.col(i);
		_tau_pd.segment(i * 3, 3) = _Kp * (_posGoal4.col(i) - pos) + _Kd * (_velGoal4.col(i) - vel);
	}
//	_tau = _tau_des + _tau_pd;
	_tau = _tau_des * 1.5;

	_lowCmd->setTau(_tau);
}