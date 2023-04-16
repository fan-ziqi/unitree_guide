#ifndef BACKFLIP_CTRL
#define BACKFLIP_CTRL

#include "DataReader.hpp"
#include "DataReadCtrl.hpp"
//#include <Dynamics/FloatingBaseModel.h>
#include "message/LowlevelCmd.h"

template<typename T>
class BackFlipCtrl : public DataReadCtrl<T>
{
public:
	BackFlipCtrl(DataReader *, float _dt);
	virtual ~BackFlipCtrl();

	virtual void OneStep(float _curr_time, bool b_preparation, LowlevelCmd *command);

protected:
	void _update_joint_command();

};

#endif
