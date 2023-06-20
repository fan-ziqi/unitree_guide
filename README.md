# Overview

The unitree_guide is an open source project for controlling the quadruped robot of Unitree Robotics, and it is also the
software project
accompanying [《四足机器人控制算法--建模、控制与实践》](https://detail.tmall.com/item.htm?spm=a212k0.12153887.0.0.5487687dBgiovR&id=704510718152)
published by Unitree Robotics.

# Quick Start

The following will quickly introduce the use of unitree_guide in the gazebo simulator. For more usage, please refer to
《四足机器人控制算法--建模、控制与实践》.

## Environment

We recommand users to run this project in Ubuntu 18.04 and ROS melodic environment.

## Dependencies

1. [unitree_guide](https://github.com/unitreerobotics/unitree_guide)<br>
2. [unitree_ros](https://github.com/unitreerobotics/unitree_ros)<br>
3. [unitree_legged_msgs](https://github.com/unitreerobotics/unitree_ros_to_real)(Note that: unitree_legged_real package
   should not be a part of dependencies)<br>

Put these three packages in the src folder of a ROS workspace.

## build

Open a terminal and switch the directory to the ros workspace containing unitree_guide, then run the following command
to build the project:

```
catkin_make
```

If you have any error in this step, you can raise an issue to us.

## run

In the same terminal, run the following command step by step:

```
source ./devel/setup.bash
```

To open the gazebo simulator, run:

```
roslaunch unitree_guide gazeboSim.launch 
```

For starting the controller, open an another terminal and switch to the same directory, then run the following command:

```
./devel/lib/unitree_guide/junior_ctrl
```

## usage

After starting the controller, the robot will lie on the ground of the simulator, then press the '2' key on the keyboard
to switch the robot's finite state machine (FSM) from **Passive**(initial state) to **FixedStand**, then press the '4'
key to switch the FSM from **FixedStand** to **Trotting**, now you can press the 'w' 'a' 's' 'd' key to control the
translation of the robot, and press the 'j' 'l' key to control the rotation of the robot. Press the Spacebar, the robot
will stop and stand on the ground
. (If there is no response, you need to click on the terminal opened for starting the controller and then repeat the
previous operation)

# Note

Unitree_guide provides a basic quadruped robot controller for beginners. To achive better performance, additional fine
tuning of parameters or more advanced methods (such as MPC etc.) might be required. Any contribution and good idea from
the robotics community are all welcome. Feel free to raise an issue ~ <br>

# 移植到Cyberdog

## 编译

```
mkdir build && cd build
cmake ..
make -j8
```

可执行文件为`build/bin/junior_ctrl`，代码使用了实时进程，请使用sudo权限执行。

## 改动

* `WirelessHandle.cpp`、`WirelessHandle.h`、`IOSDK.cpp`、`IOSDK.h`
  中增加`#if defined(ROBOT_TYPE_A1) || defined(ROBOT_TYPE_Go1)`宏定义

* 在`CMakeLists.txt`中增加`CYBERDOG`相关代码

  ```cmake
  if(${ROBOT_TYPE} STREQUAL "A1")
    add_definitions(-DROBOT_TYPE_A1)
  elseif(${ROBOT_TYPE} STREQUAL "Go1")
    add_definitions(-DROBOT_TYPE_Go1)
  elseif(${ROBOT_TYPE} STREQUAL "CYBERDOG")
    add_definitions(-DROBOT_TYPE_CYBERDOG)
  else()
  
  if(REAL_ROBOT)
    add_definitions(-DCOMPILE_WITH_REAL_ROBOT)
    if(${ROBOT_TYPE} STREQUAL "A1")
        include_directories(
                library/unitree_legged_sdk_3.2/include
        )
        link_directories(
                library/unitree_legged_sdk_3.2/lib
        )
    elseif(${ROBOT_TYPE} STREQUAL "Go1")
        include_directories(
                library/unitree_legged_sdk-3.8.0/include
        )
        if(${PLATFORM} STREQUAL "amd64")
            link_directories(
                    library/unitree_legged_sdk-3.8.0/lib/cpp/amd64
            )
        elseif(${PLATFORM} STREQUAL "arm64")
            link_directories(
                    library/unitree_legged_sdk-3.8.0/lib/cpp/arm64
            )
        endif()
    elseif(${ROBOT_TYPE} STREQUAL "CYBERDOG")
        add_subdirectory(library/cyberdog_motor_sdk)
        include_directories(
                library/cyberdog_motor_sdk/include
        )
    endif()
  endif()
  
  if(REAL_ROBOT)
    if(${ROBOT_TYPE} STREQUAL "A1")
        if(${PLATFORM} STREQUAL "amd64")
            target_link_libraries(junior_ctrl libunitree_legged_sdk_amd64.so)
        elseif(${PLATFORM} STREQUAL "arm64")
            target_link_libraries(junior_ctrl libunitree_legged_sdk_arm64.so)
        endif()
    elseif(${ROBOT_TYPE} STREQUAL "Go1")
        target_link_libraries(junior_ctrl libunitree_legged_sdk.a)
    elseif(${ROBOT_TYPE} STREQUAL "CYBERDOG")
        target_link_libraries(junior_ctrl cyberdog_motor_sdk)
    endif()
  endif()
  ```

* 新增`IOCYBERDOG.cpp`、`IOCYBERDOG.h`接口文件

  ```cpp
  #ifdef COMPILE_WITH_REAL_ROBOT
  
  #if defined(ROBOT_TYPE_CYBERDOG)
  
  #ifndef IOCYBERDOG_H
  #define IOCYBERDOG_H
  
  #include "interface/IOInterface.h"
  
  // cyberdog接口
  #include <CustomInterface.h>
  
  #ifdef COMPILE_WITH_MOVE_BASE
  #include <ros/ros.h>
  #include <ros/time.h>
  #include <sensor_msgs/JointState.h>
  #endif  // COMPILE_WITH_MOVE_BASE
  
  using CyberdogData = Robot_Data;
  using CyberdogCmd = Motor_Cmd;
  
  class IOCYBERDOG : public CustomInterface, public IOInterface
  {
  public:
  	IOCYBERDOG();
  	~IOCYBERDOG() {}
  	void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);
  
  private:
  	CyberdogData cyberdogData;
  	CyberdogCmd cyberdogCmd;
  	void UserCode();
  	long long count = 0;
  
  #ifdef COMPILE_WITH_MOVE_BASE
  	ros::NodeHandle _nh;
  	ros::Publisher _pub;
  	sensor_msgs::JointState _joint_state;
  #endif  // COMPILE_WITH_MOVE_BASE
  };
  
  #endif  // IOCYBERDOG_H
  
  #endif
  
  #endif  // COMPILE_WITH_REAL_ROBOT
  ```

  ```cpp
  #ifdef COMPILE_WITH_REAL_ROBOT
  
  #if defined(ROBOT_TYPE_CYBERDOG)
  
  #include "interface/IOCYBERDOG.h"
  #include "interface/KeyBoard.h"
  
  //#define DEBUG_MOTOR
  
  IOCYBERDOG::IOCYBERDOG()
  		: CustomInterface(500)
  {
  	std::cout << "The control interface for real robot (Cyberdog)" << std::endl;
  
  	cmdPanel = new KeyBoard();
  
  #ifdef COMPILE_WITH_MOVE_BASE
  	_pub = _nh.advertise<sensor_msgs::JointState>("/realRobot/joint_states", 20);
  	_joint_state.name.resize(12);
  	_joint_state.position.resize(12);
  	_joint_state.velocity.resize(12);
  	_joint_state.effort.resize(12);
  #endif  // COMPILE_WITH_MOVE_BASE
  }
  
  void IOCYBERDOG::sendRecv(const LowlevelCmd *cmd, LowlevelState *state)
  {
  	for(int i(0); i < 12; ++i)
  	{
  		cyberdogCmd.q_des[i] = cmd->motorCmd[i].q;
  		cyberdogCmd.qd_des[i] = cmd->motorCmd[i].dq;
  		cyberdogCmd.kp_des[i] = cmd->motorCmd[i].Kp;
  		cyberdogCmd.kd_des[i] = cmd->motorCmd[i].Kd;
  		cyberdogCmd.tau_des[i] = cmd->motorCmd[i].tau;
  	}
  
  	for(int i(0); i < 12; ++i)
  	{
  		state->motorState[i].q = cyberdogData.q[i];
  		state->motorState[i].dq = cyberdogData.qd[i];
  		state->motorState[i].tauEst = cyberdogData.tau[i];
  	}
  
  	//IMU
  	for(int i = 0; i < 3; i++)
  	{
  		state->imu.accelerometer[i] = cyberdogData.acc[i];
  	}
  	// 注意 Cyberdog SDK 的四元数顺序为 xyzw 需要转成 wxyz
  	state->imu.quaternion[0] = cyberdogData.quat[1];
  	state->imu.quaternion[1] = cyberdogData.quat[2];
  	state->imu.quaternion[2] = cyberdogData.quat[3];
  	state->imu.quaternion[3] = cyberdogData.quat[0];
  	for(int i = 0; i < 3; i++)
  	{
  		state->imu.gyroscope[i] = cyberdogData.omega[i];
  	}
  
  	state->userCmd = cmdPanel->getUserCmd();
  	state->userValue = cmdPanel->getUserValue();
  
  #ifdef COMPILE_WITH_MOVE_BASE
  	_joint_state.header.stamp = ros::Time::now();
  	_joint_state.name = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
  						 "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
  						 "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
  						 "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
  	for(int i(0); i<12; ++i){
  		_joint_state.position[i] = state->motorState[i].q;
  		_joint_state.velocity[i] = state->motorState[i].dq;
  		_joint_state.effort[i]   = state->motorState[i].tauEst;
  	}
  
  	_pub.publish(_joint_state);
  #endif  // COMPILE_WITH_MOVE_BASE
  }
  
  void IOCYBERDOG::UserCode()
  {
  	cyberdogData = robot_data;
  	motor_cmd = cyberdogCmd;
  
  #ifdef DEBUG_MOTOR
  	if((count++) % 1000 == 0)
  	{
  		printf("interval:---------%.4f-------------\n", cyberdogData.ctrl_topic_interval);
  		printf("rpy [3]:");
  		for(int i = 0; i < 3; i++)
  			printf(" %.2f", cyberdogData.rpy[i]);
  		printf("\nacc [3]:");
  		for(int i = 0; i < 3; i++)
  			printf(" %.2f", cyberdogData.acc[i]);
  		printf("\nquat[4]:");
  		for(int i = 0; i < 4; i++)
  			printf(" %.2f", cyberdogData.quat[i]);
  		printf("\nomeg[3]:");
  		for(int i = 0; i < 3; i++)
  			printf(" %.2f", cyberdogData.omega[i]);
  		printf("\nq  [12]:");
  		for(int i = 0; i < 12; i++)
  			printf(" %.2f", cyberdogData.q[i]);
  		printf("\nqd [12]:");
  		for(int i = 0; i < 12; i++)
  			printf(" %.2f", cyberdogData.qd[i]);
  		printf("\ntau[12]:");
  		for(int i = 0; i < 12; i++)
  			printf(" %.2f", cyberdogData.tau[i]);
  		printf("\nq_des[12]:");
  		for(int i = 0; i < 12; i++)
  			printf(" %.2f", cyberdogCmd.q_des[i]);
  		printf("\nqd_des[12]:");
  		for(int i = 0; i < 12; i++)
  			printf(" %.2f", cyberdogCmd.qd_des[i]);
  		printf("\nkp_des[12]:");
  		for(int i = 0; i < 12; i++)
  			printf(" %.2f", cyberdogCmd.kp_des[i]);
  		printf("\nkd_des[12]:");
  		for(int i = 0; i < 12; i++)
  			printf(" %.2f", cyberdogCmd.kd_des[i]);
  		printf("\ntau_des[12]:");
  		for(int i = 0; i < 12; i++)
  			printf(" %.2f", cyberdogCmd.tau_des[i]);
  		printf("\n\n");
  	}
  #endif // DEBUG_MOTOR
  }
  
  #endif
  
  #endif  // COMPILE_WITH_REAL_ROBOT
  ```

* 在`unitreeLeg.h`中新增`CYBERDOGLeg`子类

  ```cpp
  class CYBERDOGLeg : public QuadrupedLeg
  {
  public:
  	CYBERDOGLeg(const int legID, const Vec3 pHip2B)
  			: QuadrupedLeg(legID, 0.10715, 0.2, 0.217, pHip2B) {}
  	~CYBERDOGLeg() {}
  };
  ```

* 在`unitreeRobot.cpp`、`unitreeRobot.h`中新增`CYBERDOGRobot`子类

  ```c++
  class CYBERDOGRobot : public QuadrupedRobot
  {
  public:
  	CYBERDOGRobot();
  	~CYBERDOGRobot() {};
  };
  ```

  ```c++
  CYBERDOGRobot::CYBERDOGRobot()
  {
  	_Legs[0] = new CYBERDOGLeg(0, Vec3(0.235, -0.05, 0));
  	_Legs[1] = new CYBERDOGLeg(1, Vec3(0.235, 0.05, 0));
  	_Legs[2] = new CYBERDOGLeg(2, Vec3(-0.235, -0.05, 0));
  	_Legs[3] = new CYBERDOGLeg(3, Vec3(-0.235, 0.05, 0));
  
  	_feetPosNormalStand << 0.235, 0.235, -0.235, -0.235, //abad_x
  			-0.15715, 0.15715, -0.15715, 0.15715, //abad_y  + abad_link_length
  			-0.2800, -0.2800, -0.2800, -0.2800; // 调试出来的吗？我也不知道
  
  	_robVelLimitX << -0.4, 0.4;
  	_robVelLimitY << -0.3, 0.3;
  	_robVelLimitYaw << -0.5, 0.5;
  
  #ifdef COMPILE_WITH_REAL_ROBOT
  	_mass = 12.328; // 低，增大质量 高，减小质量
  	_pcb << 0.0, 0.0, 0.0; // 抬头后移，低头前移
  	_Ib = Vec3(0.13, 0.54, 0.63).asDiagonal();
  #endif  // COMPILE_WITH_REAL_ROBOT
  
  #ifdef COMPILE_WITH_SIMULATION
  	_mass = 12.328; // 低，增大质量 高，减小质量
  	_pcb << -0.01, 0.0, 0.0; // 抬头后移，低头前移
  	_Ib = Vec3(0.13, 0.54, 0.63).asDiagonal();
  #endif  // COMPILE_WITH_SIMULATION
  }
  
  ```

* 在`State_FreeStand.cpp`中找到`_heightMax = 0.04;`并修改机身高度
  
  ```cpp
  //	_heightMax = 0.04;
  #ifdef ROBOT_TYPE_A1
  	_heightMax = 0.4;
  #endif
  #ifdef ROBOT_TYPE_Go1
  	_heightMax = 0.4;
  #endif
  #ifdef ROBOT_TYPE_CYBERDOG
  	_heightMax = 0.4;
  #endif
  ```
  
* 在`State_Trotting.cpp`中找到`_gaitHeight = 0.08;`并修改抬腿高度，并添加参数
  
  ```cpp
  #ifdef ROBOT_TYPE_A1
  _gaitHeight = 0.08;
  #endif
  #ifdef ROBOT_TYPE_Go1
  _gaitHeight = 0.08;
  #endif
  #ifdef ROBOT_TYPE_CYBERDOG
  _gaitHeight = 0.08;
  #endif
  
  #ifdef ROBOT_TYPE_CYBERDOG
  	_Kpp = Vec3(70, 70, 70).asDiagonal();        // 位置刚度 在保持平衡的前提下尽量缩小
  	_Kdp = Vec3(10, 10, 10).asDiagonal();        // 位置阻尼
  	_kpw = 500;                                  // 姿态刚度 在保持平衡的前提下尽量缩小
  	_Kdw = Vec3(50, 50, 50).asDiagonal();        // 姿态阻尼
  	_KpSwing = Vec3(400, 400, 400).asDiagonal(); // 足端刚度
  	_KdSwing = Vec3(25, 25, 25).asDiagonal();    // 足端阻尼
  #endif
  ```
  
* 修改`LowlevelCmd.h`中的`setTau`函数
  
  ```cpp
  #ifdef ROBOT_TYPE_A1
  	void setTau(Vec12 tau, Vec2 torqueLimit = Vec2(-35, 35)){
  		for(int i(0); i<12; ++i){
  			if(std::isnan(tau(i))){
  				printf("[ERROR] The setTau function meets Nan\n");
  			}
  			motorCmd[i].tau = saturation(tau(i), torqueLimit);
  		}
  	}
  #endif
  #ifdef ROBOT_TYPE_Go1
  	void setTau(Vec12 tau, Vec2 torqueLimit = Vec2(-35, 35)){
  		for(int i(0); i<12; ++i){
  			if(std::isnan(tau(i))){
  				printf("[ERROR] The setTau function meets Nan\n");
  			}
  			motorCmd[i].tau = saturation(tau(i), torqueLimit);
  		}
  	}
  #endif
  #ifdef ROBOT_TYPE_CYBERDOG
  	void setTau(Vec12 tau, Vec2 torqueLimit = Vec2(-35, 35))
  	{
  		for(int i(0); i < 12; ++i)
  		{
  			if(std::isnan(tau(i)))
  			{
  				printf("[ERROR] The setTau function meets Nan\n");
  			}
  			motorCmd[i].tau = saturation(tau(i), torqueLimit);
  		}
  	}
  #endif
  ```
  
* 修改`LowlevelCmd.h`中的kp、kd
  
  ```cpp
  #ifdef ROBOT_TYPE_A1
  	void setSimStanceGain(int legID){
  		motorCmd[legID*3+0].mode = 10;
  		motorCmd[legID*3+0].Kp = 180;
  		motorCmd[legID*3+0].Kd = 8;
  		motorCmd[legID*3+1].mode = 10;
  		motorCmd[legID*3+1].Kp = 180;
  		motorCmd[legID*3+1].Kd = 8;
  		motorCmd[legID*3+2].mode = 10;
  		motorCmd[legID*3+2].Kp = 300;
  		motorCmd[legID*3+2].Kd = 15;
  	}
  #endif
  #ifdef ROBOT_TYPE_Go1
  	void setSimStanceGain(int legID){
  		motorCmd[legID*3+0].mode = 10;
  		motorCmd[legID*3+0].Kp = 180;
  		motorCmd[legID*3+0].Kd = 8;
  		motorCmd[legID*3+1].mode = 10;
  		motorCmd[legID*3+1].Kp = 180;
  		motorCmd[legID*3+1].Kd = 8;
  		motorCmd[legID*3+2].mode = 10;
  		motorCmd[legID*3+2].Kp = 300;
  		motorCmd[legID*3+2].Kd = 15;
  	}
  #endif
  #ifdef ROBOT_TYPE_CYBERDOG
  	void setSimStanceGain(int legID)
  	{
  		motorCmd[legID * 3 + 0].mode = 10;
  		motorCmd[legID * 3 + 0].Kp = 180;
  		motorCmd[legID * 3 + 0].Kd = 8;
  		motorCmd[legID * 3 + 1].mode = 10;
  		motorCmd[legID * 3 + 1].Kp = 180;
  		motorCmd[legID * 3 + 1].Kd = 8;
  		motorCmd[legID * 3 + 2].mode = 10;
  		motorCmd[legID * 3 + 2].Kp = 300;
  		motorCmd[legID * 3 + 2].Kd = 15;
  	}
  #endif
  ```
  
* 在`main.cpp`中增加
  
  ```cmake
  #ifdef COMPILE_WITH_REAL_ROBOT
  #if defined(ROBOT_TYPE_A1) || defined(ROBOT_TYPE_Go1)
  #include "interface/IOSDK.h"
  #elif defined(ROBOT_TYPE_CYBERDOG)
  #include "interface/IOCYBERDOG.h"
  #endif
  #endif  // COMPILE_WITH_REAL_ROBOT
  
  #ifdef COMPILE_WITH_REAL_ROBOT
  #ifdef ROBOT_TYPE_A1
  	ioInter = new IOSDK();
  	ctrlPlat = CtrlPlatform::REALROBOT;
  #endif
  #ifdef ROBOT_TYPE_Go1
  	ioInter = new IOSDK();
  	ctrlPlat = CtrlPlatform::REALROBOT;
  #endif
  #ifdef ROBOT_TYPE_CYBERDOG
  	ioInter = new IOCYBERDOG();
  	ctrlPlat = CtrlPlatform::REALROBOT;
  #endif
  #endif  // COMPILE_WITH_REAL_ROBOT
  
  #ifdef ROBOT_TYPE_A1
  	ctrlComp->robotModel = new A1Robot();
  #endif
  #ifdef ROBOT_TYPE_Go1
  	ctrlComp->robotModel = new Go1Robot();
  #endif
  #ifdef ROBOT_TYPE_CYBERDOG
  	ctrlComp->robotModel = new CYBERDOGRobot();
  #endif
  ```