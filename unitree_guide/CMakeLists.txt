cmake_minimum_required(VERSION 3.14)
project(unitree_guide)

set(ROBOT_TYPE CYBERDOG)    # The type of robot, support Go1 and A1 currently && CYBERDOG
set(PLATFORM amd64)         # The platform to compile, support amd64 and arm64
set(CATKIN_MAKE ON)        # Use CATKIN_MAKE or not, ON or OFF
set(SIMULATION ON)         # Use Gazebo or not, ON or OFF
set(REAL_ROBOT OFF)          # Link real robot or not, ON or OFF
set(DEBUG OFF)              # Use debug functions or not, ON or OFF
set(MOVE_BASE OFF)          # Need move_base or not, ON or OFF

message("ROBOT_TYPE: " ${ROBOT_TYPE})
message("PLATFORM: " ${PLATFORM})
message("CATKIN_MAKE: " ${CATKIN_MAKE})
message("SIMULATION: " ${SIMULATION})
message("REAL_ROBOT: " ${REAL_ROBOT})
message("DEBUG: " ${DEBUG})

if(NOT DEFINED ROBOT_TYPE)
    message(FATAL_ERROR "[CMake ERROR] Have not defined ROBOT_TYPE")
endif()
if(NOT DEFINED PLATFORM)
    message(FATAL_ERROR "[CMake ERROR] Have not defined PLATFORM")
endif()

if(${ROBOT_TYPE} STREQUAL "A1")
    add_definitions(-DROBOT_TYPE_A1)
elseif(${ROBOT_TYPE} STREQUAL "Go1")
    add_definitions(-DROBOT_TYPE_Go1)
elseif(${ROBOT_TYPE} STREQUAL "CYBERDOG")
    add_definitions(-DROBOT_TYPE_CYBERDOG)
else()
    message(FATAL_ERROR "[CMake ERROR] The ROBOT_TYPE is error")
endif()

if(((SIMULATION) AND (REAL_ROBOT)) OR ((NOT SIMULATION) AND (NOT REAL_ROBOT)))
    message(FATAL_ERROR "[CMake ERROR] The SIMULATION and REAL_ROBOT can only be one ON one OFF")
endif()

if(SIMULATION OR MOVE_BASE)
    add_definitions(-DRUN_ROS)
    set(CATKIN_MAKE ON)
endif()

set(CMAKE_CXX_STANDARD 11)
find_package(Boost)

if(CATKIN_MAKE)
    add_definitions(-DCOMPILE_WITH_ROS)
    if(MOVE_BASE)
        add_definitions(-DCOMPILE_WITH_MOVE_BASE)
    endif()
endif()

if(DEBUG)
    add_definitions(-DCOMPILE_DEBUG)
    find_package(Python2 COMPONENTS Interpreter Development NumPy)
endif()

if(CATKIN_MAKE)
    if(SIMULATION)
        add_definitions(-DCOMPILE_WITH_SIMULATION)
        find_package(catkin REQUIRED COMPONENTS
                controller_manager
                joint_state_controller
                gazebo_ros
                gazebo_msgs
                gazebo_plugins
                gazebo_ros_control
                )
    endif()

    find_package(catkin REQUIRED COMPONENTS
            genmsg
            robot_state_publisher
            roscpp
            std_msgs
            tf
            geometry_msgs
            unitree_legged_msgs

            xpp_msgs #towr
            rosbag #towr
            )
    catkin_package(
            CATKIN_DEPENDS
            unitree_legged_msgs

            xpp_msgs #towr
    )
endif()

include_directories(
        include
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -O3")

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

link_directories(
        ${GAZEBO_LIBRARY_DIRS}
)

# aux_source_directory(src SRC_LIST)
file(GLOB_RECURSE SRC_LIST
        "src/*/*.cpp"
        "src/*/*.cc"
        "src/*/*/*.cpp"
        )

add_executable(junior_ctrl src/main.cpp ${SRC_LIST})
if(CATKIN_MAKE)
    target_link_libraries(junior_ctrl ${catkin_LIBRARIES})
    add_dependencies(junior_ctrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif()
if(DEBUG)
    target_include_directories(junior_ctrl PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})
    target_link_libraries(junior_ctrl Python2::Python Python2::NumPy)
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
target_link_libraries(junior_ctrl -pthread lcm)

if(NOT CATKIN_MAKE)
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/bin)
endif()

## PlotTest
#add_executable(myTest src/test.cpp ${SRC_LIST})
#if(CATKIN_MAKE)
#    target_link_libraries(myTest ${catkin_LIBRARIES})
#    add_dependencies(myTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#endif()
#find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
#target_link_libraries(myTest
#        Python3::Python
#        Python3::Module
#        )
#find_package(Python3 COMPONENTS NumPy)
#if(Python3_NumPy_FOUND)
#    target_link_libraries(myTest
#            Python3::NumPy
#            )
#else()
#    target_compile_definitions(myTest WITHOUT_NUMPY)
#endif()
#if(REAL_ROBOT)
#    if(${ROBOT_TYPE} STREQUAL "A1")
#        if(${PLATFORM} STREQUAL "amd64")
#            target_link_libraries(myTest libunitree_legged_sdk_amd64.so)
#        elseif(${PLATFORM} STREQUAL "arm64")
#            target_link_libraries(myTest libunitree_legged_sdk_arm64.so)
#        endif()
#    elseif(${ROBOT_TYPE} STREQUAL "Go1")
#        target_link_libraries(myTest libunitree_legged_sdk.a)
#    elseif(${ROBOT_TYPE} STREQUAL "CYBERDOG")
#        target_link_libraries(myTest cyberdog_motor_sdk)
#    endif()
#endif()
#target_link_libraries(myTest -pthread lcm)


#find_package(catkin REQUIRED COMPONENTS
#        roscpp
#        rosbag
#        std_msgs
#        visualization_msgs
#        message_generation
#        xpp_msgs
#        )
#catkin_package(
#        CATKIN_DEPENDS xpp_msgs
#)
#include_directories(
#        include
#        ${catkin_INCLUDE_DIRS}
#)
#add_executable(read_bag src/read_bag.cpp)
#target_link_libraries(read_bag ${catkin_LIBRARIES})
#add_dependencies(read_bag ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})
target_link_libraries(junior_ctrl yaml-cpp)