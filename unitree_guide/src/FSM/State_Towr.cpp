#include "FSM/State_Towr.h"
#include <yaml-cpp/yaml.h>

//#define TOWR_DEBUG

#define YAML_PATH "/mnt/f/我的文件/研究生/四足项目/宇树/unitree_guide/src/unitree_guide/unitree_guide/src/FSM/towr_config.yaml"

State_Towr::State_Towr(CtrlComponents *ctrlComp)
		: FSMState(ctrlComp, FSMStateName::TOWR, "Towr")
{
	// 关闭安全检测
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

	_Kp = Vec3(100, 100, 100).asDiagonal();
	_Kd = Vec3(10, 10, 10).asDiagonal();

	for(int i = 0; i < 12; i++)
	{
		_lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
	}

//	_ctrlComp->setAllSwing();

	// TOWR START

	std::string src_bag = "/mnt/f/我的文件/研究生/四足项目/宇树/unitree_guide/src/thirdparty/xpp_msgs/bags/cyberdog_bag_13263.bag";
	std::string state_topic = "/xpp/state_des";

	rosbag::Bag bag;
	bag.open(src_bag, rosbag::bagmode::Read);
	std::vector<std::string> topics;
	topics.push_back(std::string(state_topic));
	rosbag::View view(bag, rosbag::TopicQuery(topics));

	// 在towr中，左右腿的序号是反的，在这里需要对其反向。即0123对应1032
	int ee_num;
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

			_basePosePos << state->base.pose.position.x, state->base.pose.position.y, state->base.pose.position.z;
			_basePosePosVec.push_back(_basePosePos);
			_basePoseOri
					<< state->base.pose.orientation.x, state->base.pose.orientation.y, state->base.pose.orientation.z, state->base.pose.orientation.w;
			_basePoseOriVec.push_back(_basePoseOri);

			_baseTwiLin << state->base.twist.linear.x, state->base.twist.linear.y, state->base.twist.linear.z;
			_baseTwiLinVec.push_back(_baseTwiLin);
			_baseTwiAng << state->base.twist.angular.x, state->base.twist.angular.y, state->base.twist.angular.z;
			_baseTwiAngVec.push_back(_baseTwiAng);

			_baseAccLin << state->base.accel.linear.x, state->base.accel.linear.y, state->base.accel.linear.z;
			_baseAccLinVec.push_back(_baseAccLin);
			_baseAccAng << state->base.accel.angular.x, state->base.accel.angular.y, state->base.accel.angular.z;
			_baseAccAngVec.push_back(_baseAccAng);

			ee_num = 0;
			for(auto &ee: state->ee_motion)
			{
				_eePos4.col(ee_index[ee_num]) << ee.pos.x, ee.pos.y, ee.pos.z;
				// 限制
				if(_eePos4.col(ee_index[ee_num])(2) > 0.2) _eePos4.col(ee_index[ee_num])(2) = 0;
				_eeVel4.col(ee_index[ee_num]) << ee.vel.x, ee.vel.y, ee.vel.z;
				_eeAcc4.col(ee_index[ee_num]) << ee.acc.x, ee.acc.y, ee.acc.z;
				ee_num++;
			}
			_eePos4Vec.push_back(_eePos4);
			_eeVel4Vec.push_back(_eeVel4);
			_eeVel4Acc.push_back(_eeAcc4);

			ee_num = 0;
			for(auto &ee_force: state->ee_forces)
			{
				_eeForce4.col(ee_index[ee_num]) << ee_force.x, ee_force.y, ee_force.z;
				ee_num++;
			}
			_eeForce4Vec.push_back(_eeForce4);

			ee_num = 0;
			for(auto &contact: state->ee_contact)
			{
				if(contact == true)
				{
					_eeContact4(ee_index[ee_num]) = 1;
				}
				else
				{
					_eeContact4(ee_index[ee_num]) = 0;
				}
				ee_num++;
			}
			_eeContact4Vec.push_back(_eeContact4);
		}
	}
	bag.close();

	read_yaml();

#ifdef TOWR_DEBUG
	for(int index = 0; index < _eePos4Vec.size() - 1; ++index)
	{
		Vec34 eePos4 = _eePos4Vec.at(index);
		Vec3 basePosePos = _basePosePosVec.at(index);
		for(int i = 0; i < 4; ++i)
		{
			for(int j = 0; j < 3; ++j)
			{
				std::cout << eePos4(j, i) - basePosePos(j) << " ";
			}
			std::cout << std::endl;
		}
		std::cout << std::endl;
	}
	for(const auto &torGoal4: _eeForce4Vec)
	{
		for(int i = 0; i < 3; ++i)
		{
			for(int j = 0; j < 4; ++j)
			{
				std::cout << torGoal4(i, j) << " ";
			}
			std::cout << std::endl;
		}
		std::cout << std::endl;
	}
	for(const auto &basePos: _basePosePosVec)
	{
		for(int i = 0; i < 3; ++i)
		{
			std::cout << basePos(i) << " ";
		}
		std::cout << std::endl;
	}
	for(const auto &eeContact: _eeContact4Vec)
	{
		for(int i = 0; i < 3; ++i)
		{
			std::cout << eeContact(i) << " ";
		}
		std::cout << std::endl;
	}
#endif

	_ctrlComp->ioInter->zeroCmdPanel();
	_ctrl_index = 0;
	last_time = std::chrono::steady_clock::now();
}

void State_Towr::run()
{
	int update_interval = 10; // 更新间隔，单位为毫秒

	_eeForce4 = _eeForce4Vec.at(_ctrl_index);
	_eePos4 = _eePos4Vec.at(_ctrl_index);
	_eeVel4 = _eeVel4Vec.at(_ctrl_index);
	_eeContact4 = _eeContact4Vec.at(_ctrl_index);
	_basePosePos = _basePosePosVec.at(_ctrl_index);

	for(int i = 0; i < 4; ++i)
	{
		if(_eeContact4(i) == 0)
		{
			_lowCmd->setSwingGain(i);
		}
		else
		{
			_lowCmd->setStableGain(i);
		}
	}

	// 力矩控制
	_torqueCtrl();

	// 定时10ms控制
	current_time = std::chrono::steady_clock::now();
	auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count();

	if(elapsed_ms >= update_interval)
	{
		if(_ctrl_index < _eeForce4Vec.size() - 1)
		{
			_ctrl_index++;
		}
		else
		{
			// 执行结束，力矩设为0
			_eeForce4.setZero();
		}
		last_time = std::chrono::steady_clock::now();
	}

}

void State_Towr::exit()
{
	_eeForce4Vec.clear();
	_eePos4Vec.clear();
	_eeContact4Vec.clear();
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

void State_Towr::_torqueCtrl()
{
	Vec3 eePos_now, eePos_goal, eeVel_now;
	Mat3 jaco;
	Vec12 tau_des, tau_pd, tau;

	for(int i = 0; i < 4; i++)
	{
		eePos_now = _ctrlComp->robotModel->getFootPosition(*_lowState, i, FrameType::BODY);
		eeVel_now = _ctrlComp->robotModel->getFootVelocity(*_lowState, i);
		jaco = _ctrlComp->robotModel->getJaco(*_lowState, i);

		// towr的足端位置是在世界坐标系下表示的，需要进行坐标变换
		eePos_goal = _eePos4.col(i) - _basePosePos;

		tau_des.segment(i * 3, 3) = jaco.transpose() * -_eeForce4.col(i);
		tau_pd.segment(i * 3, 3) = jaco.transpose() * (_Kp * (eePos_goal - eePos_now) + _Kd * (-eeVel_now));
	}
	tau = tau_des * 1.0 + tau_pd;
//	std::cout << tau[0] << " " << tau[1] << " " << tau[2] << std::endl;

	_lowCmd->setTau(tau);
}

void State_Towr::read_yaml()
{
	YAML::Node config;
	try
	{
		config = YAML::LoadFile(YAML_PATH);
	} catch(YAML::BadFile &e)
	{
		std::cout << "read error, please change YAML_PATH in file State_Towr.cpp" << std::endl;
		return;
	}

	_Kp = Vec3(config["kp"]["a"].as<int>(), config["kp"]["b"].as<int>(), config["kp"]["c"].as<int>()).asDiagonal();
	_Kd = Vec3(config["kd"]["a"].as<int>(), config["kd"]["b"].as<int>(), config["kd"]["c"].as<int>()).asDiagonal();

	std::cout << "Change kp to ["
	          << config["kp"]["a"].as<int>() << ", "
	          << config["kp"]["b"].as<int>() << ", "
	          << config["kp"]["c"].as<int>() << ", "
	          << "]" << std::endl;
	std::cout << "Change kd to ["
	          << config["kd"]["a"].as<int>() << ", "
	          << config["kd"]["b"].as<int>() << ", "
	          << config["kd"]["c"].as<int>() << ", "
	          << "]" << std::endl;
}
