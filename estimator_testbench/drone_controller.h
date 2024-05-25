#pragma once

#include <vector>

#include "environmental_model.h"

class Action
{
public:
	virtual void doit(SimpleDrone* drone) = 0;
};

class RollAction 
{
	float rads;

	RollAction(float rads)
	{
		this->rads = rads;
	}

	void doit(SimpleDrone *drone)
	{
		drone->roll_rads(rads);
	}
};

class PitchAction
{
	float rads;

	PitchAction(float rads)
	{
		this->rads = rads;
	}

	void doit(SimpleDrone* drone)
	{
		drone->pitch_rads(rads);
	}
};

class YawAction
{
	float rads;

	YawAction(float rads)
	{
		this->rads = rads;
	}

	void doit(SimpleDrone* drone)
	{
		drone->yaw_rads(rads);
	}
};

using frame_action = std::vector<Action*>;

class DroneController
{

private:

public:
	void set_rotational_vibration(float amplitude, float variance);
	void set_planar_vibration(float amplitude, float variance);

	void perform_actions(std::vector<frame_action> scene, SimpleDrone *drone);

	void run_simulation();
};