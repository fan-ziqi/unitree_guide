#include "control/BackFlip/DataReader.hpp"
//#include <Configuration.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// 请修改这个路径
#define THIS_COM "/mnt/f/我的文件/研究生/四足项目/宇树/unitree_guide/"

DataReader::DataReader(const RobotType &type, FSMStateName stateNameIn)
		: _type(type)
{

	if(stateNameIn == FSMStateName::BACKFLIP)
	{
		load_control_plan(THIS_COM "src/unitree_guide/unitree_guide/src/control/BackFlip/mc_flip.dat");
		printf("[Backflip DataReader] Setup for mini cheetah\n");
	}
//    else if (stateNameIn == FSM_StateName::FRONTJUMP) {
//      //load_control_plan(THIS_COM "user/MIT_Controller/Controllers/FrontJump/front_jump_data.dat"); // front_jump_data.dat for succesfull test 1 file
//      load_control_plan(THIS_COM "config/front_jump_pitchup_v2.dat");
//      printf("[Front Jump DataReader] Setup for mini cheetah\n");
//    }

	printf("[Backflip DataReader] Constructed.\n");
}

void DataReader::load_control_plan(const char *filename)
{
	printf("[Backflip DataReader] Loading control plan %s...\n", filename);
	FILE *f = fopen(filename, "rb");
	if(!f)
	{
		printf("[Backflip DataReader] Error loading control plan!\n");
		return;
	}
	fseek(f, 0, SEEK_END);
	uint64_t file_size = ftell(f);
	fseek(f, 0, SEEK_SET);

	printf("[Backflip DataReader] Allocating %ld bytes for control plan\n",
	       file_size);

	plan_buffer = (float *) malloc(file_size + 1);

	if(!plan_buffer)
	{
		printf("[Backflip DataReader] malloc failed!\n");
		return;
	}

	uint64_t read_success = fread(plan_buffer, file_size, 1, f);
	if(!read_success)
	{
		printf("[Backflip DataReader] Error: fread failed.\n");
	}

	if(file_size % sizeof(float))
	{
		printf(
				"[Backflip DataReader] Error: file size isn't divisible by size of "
				"float!\n");
	}

	fclose(f);

	plan_loaded = true;
	plan_timesteps = file_size / (sizeof(float) * plan_cols);
	printf("[Backflip DataReader] Done loading plan for %d timesteps\n",
	       plan_timesteps);
}

float *DataReader::get_initial_configuration()
{
	if(!plan_loaded)
	{
		printf(
				"[Backflip DataReader] Error: get_initial_configuration called without "
				"a plan!\n");
		return nullptr;
	}

	return plan_buffer + 3;
}

float *DataReader::get_plan_at_time(int timestep)
{
	if(!plan_loaded)
	{
		printf(
				"[Backflip DataReader] Error: get_plan_at_time called without a "
				"plan!\n");
		return nullptr;
	}

	if(timestep < 0 || timestep >= plan_timesteps)
	{
		printf(
				"[Backflip DataReader] Error: get_plan_at_time called for timestep %d\n"
				"\tmust be between 0 and %d\n",
				timestep, plan_timesteps - 1);
		timestep = plan_timesteps - 1;
		// return nullptr;
		// recover from this!
	}

	// if(timestep < 0) { return plan_buffer + 3; }
	// if(timestep >= plan_timesteps){ timestep = plan_timesteps-1; }

	return plan_buffer + plan_cols * timestep;
}

void DataReader::unload_control_plan()
{
	free(plan_buffer);
	plan_timesteps = -1;
	plan_loaded = false;
	printf("[Backflip DataReader] Unloaded plan.\n");
}
