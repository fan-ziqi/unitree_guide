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

# 移植

* `WirelessHandle.cpp`、`WirelessHandle.h`、`IOSDK.cpp`、`IOSDK.h`
  中增加`#if defined(ROBOT_TYPE_A1) || defined(ROBOT_TYPE_Go1)`宏定义
* 中增加
  ```cmake
  if(${ROBOT_TYPE} STREQUAL "A1")
    add_definitions(-DROBOT_TYPE_A1)
  elseif(${ROBOT_TYPE} STREQUAL "Go1")
    add_definitions(-DROBOT_TYPE_Go1)
  elseif(${ROBOT_TYPE} STREQUAL "Mi")
    add_definitions(-DROBOT_TYPE_Mi)
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
    elseif(${ROBOT_TYPE} STREQUAL "Mi")
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
    elseif(${ROBOT_TYPE} STREQUAL "Mi")
        target_link_libraries(junior_ctrl cyberdog_motor_sdk)
    endif()
  endif()
  ```
* 新增`IOMI.cpp`、`IOMI.h`接口文件
* `main.cpp`中增加
  ```cmake
    #ifdef COMPILE_WITH_REAL_ROBOT
    #ifdef ROBOT_TYPE_A1
    ioInter = new IOSDK();
    ctrlPlat = CtrlPlatform::REALROBOT;
    #endif
    #ifdef ROBOT_TYPE_Go1
    ioInter = new IOSDK();
    ctrlPlat = CtrlPlatform::REALROBOT;
    #endif
    #ifdef ROBOT_TYPE_Mi
    ioInter = new IOMI();
    ctrlPlat = CtrlPlatform::REALROBOT;
    #endif
    #endif  // COMPILE_WITH_REAL_ROBOT
  
    #ifdef ROBOT_TYPE_A1
    ctrlComp->robotModel = new A1Robot();
    #endif
    #ifdef ROBOT_TYPE_Go1
    ctrlComp->robotModel = new Go1Robot();
    #endif
    #ifdef ROBOT_TYPE_Mi
    ctrlComp->robotModel = new MiRobot();
    #endif
  ```