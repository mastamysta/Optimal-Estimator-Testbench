#pragma once

#include <vector>

#include "environmental_model.h"

class Action
{
public:
	virtual void doit(SimpleDrone& drone) = 0;
};

class RollAction : Action
{
	float rads;

public:
	RollAction(float rads)
	{
		this->rads = rads;
	}

	void doit(SimpleDrone& drone)
	{
		drone.roll_rads(rads);
	}
};

class PitchAction : Action
{
	float rads;

public:
	PitchAction(float rads)
	{
		this->rads = rads;
	}

	void doit(SimpleDrone& drone)
	{
		drone.pitch_rads(rads);
	}
};

class YawAction : Action
{
	float rads;

public:
	YawAction(float rads)
	{
		this->rads = rads;
	}

	void doit(SimpleDrone& drone)
	{
		drone.yaw_rads(rads);
	}
};

class RepeatAction : Action
{
	Action *act;
	size_t times;

public:
	RepeatAction(Action* act, size_t times)
	{
		this->act = act;
		this->times = times;
	}

	void doit(SimpleDrone& drone)
	{
		for (int i = 0; i < times; i++)
			act->doit(drone);
	}
};

using delta = std::vector<Action*>;
using scene = std::vector<delta>;

enum logging_type
{
	LOGGING_NONE,
	LOGGING_CSV
};

class DroneController
{

private:

	logging_type logging = LOGGING_NONE;
	std::string logging_path;

	// Lets try our best to maintain some locality;
	std::vector<std::string> log_data;

	void perform_actions(scene scene, SimpleDrone& drone);
	void do_logging();
	void done();

public:

	DroneController(logging_type logging)
	{
		this->logging = logging;
	}

	void set_logging_path(const std::string& path) { logging_path = path; };

	void set_rotational_vibration(float amplitude, float variance);
	void set_planar_vibration(float amplitude, float variance);


	void run_simulation(SimpleDrone& drone, scene& scene);
};