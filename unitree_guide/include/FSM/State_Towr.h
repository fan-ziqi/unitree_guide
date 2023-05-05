// MIT License
//
// Copyright (c) 2023 Fan Ziqi
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef TOWR_H
#define TOWR_H

#include "FSM/FSMState.h"

class State_Towr : public FSMState
{
public:
	State_Towr(CtrlComponents *ctrlComp);
	~State_Towr() {};
	void enter();
	void run();
	void exit();
	FSMStateName checkChange();
private:
	void _positionCtrl();
	void _torqueCtrl();

	Vec34 _initFeetPos, _feetPos;
	Vec3 _initPos, _posGoal;
	Vec12 _targetPos;
	float _xMin, _xMax;
	float _yMin, _yMax;
	float _zMin, _zMax;
	Mat3 _Kp, _Kd;

};


#endif //TOWR_H
