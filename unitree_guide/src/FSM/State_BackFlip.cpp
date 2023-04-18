#include <iostream>
#include "FSM/State_BackFlip.h"

State_BackFlip::State_BackFlip(CtrlComponents *ctrlComp)
		: FSMState(ctrlComp, FSMStateName::BACKFLIP, "back flip")
{
	// 后空翻关闭安全检测
	this->checkSafeOrientation = false;

	zero_vec3.setZero();
	f_ff << 0.f, 0.f, -25.f;
}

void State_BackFlip::enter()
{
	// 与mit不同，把DataReader放到这里了，防止FSM一运行就有输出
	_data_reader = new DataReader(RobotType::CYBERDOG, FSMStateName::BACKFLIP);
	backflip_ctrl_ = new BackFlipCtrl<float>(_data_reader, 0.002);
	backflip_ctrl_->SetParameter();

	// Reset iteration counter
	iter = 0;
	_state_iter = 0;
	_count = 0;
	_curr_time = 0;
	_motion_start_iter = 0;
	_b_first_visit = true;

	// initial configuration, position
	for(int leg = 0; leg < 4; ++leg)
	{
		for(int jidx = 0; jidx < 3; ++jidx)
		{
			initial_jpos[leg][jidx] = _lowState->motorState[leg * 3 + jidx].q;
		}
	}
}

void State_BackFlip::run()
{
	// Command Computation
	if(_b_running)
	{
		if(!_Initialization())
		{
			ComputeCommand();
		}
	}
	else
	{
		_SafeCommand();
	}

	++_count;
	_curr_time += 0.002;

}

void State_BackFlip::exit()
{

}

FSMStateName State_BackFlip::checkChange()
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
		return FSMStateName::BACKFLIP;
	}
}

bool State_BackFlip::_Initialization()
{
	if(_count < _waiting_count)
	{
		for(int leg = 0; leg < 4; ++leg)
		{
			for(int jidx = 0; jidx < 3; ++jidx)
			{
				_lowCmd->motorCmd[leg * 3 + jidx].q = initial_jpos[leg][jidx];
				_lowCmd->motorCmd[leg * 3 + jidx].tau = 0.;
				_lowCmd->motorCmd[leg * 3 + jidx].dq = 0.;
				_lowCmd->motorCmd[leg * 3 + jidx].Kp = 20.;
				_lowCmd->motorCmd[leg * 3 + jidx].Kd = 2.;
			}
		}
		return true;
	}

	return false;
}

void State_BackFlip::ComputeCommand()
{
	if(_b_first_visit)
	{
		backflip_ctrl_->FirstVisit(_curr_time);
		_b_first_visit = false;
	}

	backflip_ctrl_->OneStep(_curr_time, false, _lowCmd);

	if(backflip_ctrl_->EndOfPhase(_lowState))
	{
		backflip_ctrl_->LastVisit();
	}
}

void State_BackFlip::_SafeCommand()
{
	for(int leg = 0; leg < 4; ++leg)
	{
		for(int jidx = 0; jidx < 3; ++jidx)
		{
			_lowCmd->motorCmd[leg * 3 + jidx].tau = 0.;
			_lowCmd->motorCmd[leg * 3 + jidx].q = _lowState->motorState[leg * 3 + jidx].q;
			_lowCmd->motorCmd[leg * 3 + jidx].dq = 0.;
		}
	}
}