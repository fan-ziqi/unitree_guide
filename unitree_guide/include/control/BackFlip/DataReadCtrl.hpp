#ifndef DATAREAD_CTRL
#define DATAREAD_CTRL

#include "control/BackFlip/DataReader.hpp"
//#include <Dynamics/FloatingBaseModel.h>
#include "message/LowlevelState.h"
#include "message/LowlevelCmd.h"

#define DataCtrl DataReadCtrl<T>

template<typename T>
class DataReadCtrl
{
public:
	DataReadCtrl(DataReader *data_reader, float _dt)
	{
		_data_reader = data_reader;
		dt = _dt;
		_key_pt_step = ceil(dt * 1000);

		printf("dt: %f, step:%d\n", dt, _key_pt_step);
		_Kp.resize(12);
		_Kd.resize(12);
		_des_jpos.resize(12);
		_des_jvel.resize(12);
		_jtorque.resize(12);
		_Kp_joint.resize(3);
		_Kd_joint.resize(3);
	}

	~DataReadCtrl() {};

	virtual void OneStep(float _curr_time, bool b_preparation, LowlevelCmd *command) = 0;

	void FirstVisit(float _curr_time)
	{
		_ctrl_start_time = _curr_time;
		current_iteration = 0;
		pre_mode_count = 0;
	}

	void LastVisit() {}

	bool EndOfPhase(LowlevelState *data)
	{
		if(_state_machine_time > (_end_time - 2. * dt))
		{
			return true;
		}
		for(int leg(0); leg < 4; ++leg)
		{
			if(_state_machine_time > 2.7 && -data->motorState[leg * 3 + 1].q >
			                                _q_knee_max && data->motorState[leg * 3 + 1].dq > _qdot_knee_max)
			{
				printf("Contact detected at leg [%d] => Switch to the landing phase !!! \n", leg);
				printf("state_machine_time: %lf \n", _state_machine_time);
				printf("Q-Knee: %lf \n", -data->motorState[leg * 3 + 1].q);
				printf("Qdot-Knee: %lf \n", -data->motorState[leg * 3 + 1].dq);
				return true;
			}
		}
		return false;
	}

	void SetParameter()
	{
		for(int i = 0; i < 12; i++)
		{
			_Kp[i] = 1000;
			_Kd[i] = 5.;
		}
		//_Kp_joint = {20.0, 20.0, 20.0};
		//_Kd_joint = {2.0, 2.0, 2.0};
		_Kp_joint = {10.0, 10.0, 10.0};
		_Kd_joint = {1.0, 1.0, 1.0};
	}

protected:
	DataReader *_data_reader;

	DVec<T> _Kp, _Kd;
	DVec<T> _des_jpos;
	DVec<T> _des_jvel;
	DVec<T> _jtorque;

	T dt;

	std::vector<T> _Kp_joint, _Kd_joint;

	bool _b_Preparation = false;

	bool _b_set_height_target;
	T _end_time = 5.5;
	int _dim_contact;

	T _ctrl_start_time;
	T _q_knee_max = 2.0;
	T _qdot_knee_max = 2.0;

	T _state_machine_time;

	int _key_pt_step = 1;
	int current_iteration, pre_mode_count;
};

#endif
