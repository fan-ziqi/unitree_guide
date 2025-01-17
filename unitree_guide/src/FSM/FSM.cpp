/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/FSM.h"
#include <iostream>

FSM::FSM(CtrlComponents *ctrlComp)
		: _ctrlComp(ctrlComp)
{

	_stateList.invalid = nullptr;
	_stateList.passive = new State_Passive(_ctrlComp);
	_stateList.fixedStand = new State_FixedStand(_ctrlComp);
	_stateList.freeStand = new State_FreeStand(_ctrlComp);
	_stateList.trotting = new State_Trotting(_ctrlComp);
	_stateList.balanceTest = new State_BalanceTest(_ctrlComp);
	_stateList.swingTest = new State_SwingTest(_ctrlComp);
	_stateList.stepTest = new State_StepTest(_ctrlComp);
	_stateList.backFlip = new State_BackFlip(_ctrlComp);
	_stateList.towr = new State_Towr(_ctrlComp);
#ifdef COMPILE_WITH_MOVE_BASE
	_stateList.moveBase = new State_move_base(_ctrlComp);
#endif  // COMPILE_WITH_MOVE_BASE
	initialize();
}

FSM::~FSM()
{
	_stateList.deletePtr();
}

void FSM::initialize()
{
	_currentState = _stateList.passive;
	_currentState->enter();
	_nextState = _currentState;
	_mode = FSMMode::NORMAL;
}

void FSM::run()
{
	_startTime = getSystemTime();
	_ctrlComp->sendRecv();
	_ctrlComp->runWaveGen();
	_ctrlComp->estimator->run();

	// 测试后空翻，可能需要暂时关闭安全检测
	if(!checkSafty())
	{
		_ctrlComp->ioInter->setPassive();
	}

	if(_mode == FSMMode::NORMAL)
	{
		_currentState->run();
		_nextStateName = _currentState->checkChange();
		if(_nextStateName != _currentState->_stateName)
		{
			_mode = FSMMode::CHANGE;
			_nextState = getNextState(_nextStateName);
			std::cout << "Switched from " << _currentState->_stateNameString
			          << " to " << _nextState->_stateNameString << std::endl;
		}
	}
	else if(_mode == FSMMode::CHANGE)
	{
		_currentState->exit();
		_currentState = _nextState;
		_currentState->enter();
		_mode = FSMMode::NORMAL;
		_currentState->run();
	}

	absoluteWait(_startTime, (long long) (_ctrlComp->dt * 1000000));
}

FSMState *FSM::getNextState(FSMStateName stateName)
{
	switch(stateName)
	{
		case FSMStateName::INVALID:
			return _stateList.invalid;
			break;
		case FSMStateName::PASSIVE:
			return _stateList.passive;
			break;
		case FSMStateName::FIXEDSTAND:
			return _stateList.fixedStand;
			break;
		case FSMStateName::FREESTAND:
			return _stateList.freeStand;
			break;
		case FSMStateName::TROTTING:
			return _stateList.trotting;
			break;
		case FSMStateName::BALANCETEST:
			return _stateList.balanceTest;
			break;
		case FSMStateName::SWINGTEST:
			return _stateList.swingTest;
			break;
		case FSMStateName::STEPTEST:
			return _stateList.stepTest;
			break;
		case FSMStateName::BACKFLIP:
			return _stateList.backFlip;
			break;
		case FSMStateName::TOWR:
			return _stateList.towr;
			break;
#ifdef COMPILE_WITH_MOVE_BASE
			case FSMStateName::MOVE_BASE:
				return _stateList.moveBase;
				break;
#endif  // COMPILE_WITH_MOVE_BASE
		default:
			return _stateList.invalid;
			break;
	}
}

bool FSM::checkSafty()
{
	// The angle with z axis less than 60 degree
//	// DEBUG
//	std::cout << "getRotMat: " << std::endl;
//	std::cout << _ctrlComp->lowState->getRotMat() << std::endl;
//	std::cout << std::endl;
	if(_currentState->checkSafeOrientation)
	{
		if(_ctrlComp->lowState->getRotMat()(2, 2) < 0.5)
		{
			std::cout << "broken: Orientation Safety Ceck FAIL" << std::endl;
			return false;
		}
		else
		{
			return true;
		}
	}
	else
	{
		return true;
	}
}