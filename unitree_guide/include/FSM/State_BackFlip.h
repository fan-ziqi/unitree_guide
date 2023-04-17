#ifndef BACKFLIP_H
#define BACKFLIP_H

#include "FSM/FSMState.h"
#include <control/BackFlip/DataReader.hpp>
#include <control/BackFlip/BackFlipCtrl.hpp>

class State_BackFlip : public FSMState
{
public:
	State_BackFlip(CtrlComponents *ctrlComp);
	~State_BackFlip() {}
	void enter();
	void run();
	void exit();
	FSMStateName checkChange();

private:
	// Keep track of the control iterations
	int iter = 0;
	int _motion_start_iter = 0;

	static constexpr int Preparation = 0;
	static constexpr int Flip = 1;
	static constexpr int Landing = 2;

	unsigned long long _state_iter;
	int _flag = Preparation;

	// JPos
	Vec3 initial_jpos[4];
	Vec3 zero_vec3;
	Vec3 f_ff;

	void _SetJPosInterPts(
			const size_t &curr_iter, size_t max_iter, int leg,
			const Vec3 &ini, const Vec3 &fin);

	DataReader *_data_reader;
	bool _b_running = true;
	bool _b_first_visit = true;
	int _count = 0;
	int _waiting_count = 6;
	float _curr_time = 0;
	BackFlipCtrl<float> *backflip_ctrl_;

	void SetTestParameter(const std::string &test_file);
	bool _Initialization();
	void ComputeCommand();
	void _SafeCommand();

};

#endif  // BACKFLIP_H