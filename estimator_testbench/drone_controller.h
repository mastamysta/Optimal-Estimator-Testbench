#pragma once

#include <vector>

#include "environmental_model.h"

class Action
{
	virtual void doit(SimpleDrone* drone) = 0;
	virtual void accept(SceneVisitor* visitor, SimpleDrone* drone) = 0;
};

class SceneVisitor
{
public:

	void visit(RollAction *act, SimpleDrone* drone)
	{

	}

	void visit(YawAction *act, SimpleDrone* drone)
	{

	}

	void visit(PitchAction *act, SimpleDrone* drone)
	{
		act->doit();
	}

	void visit(frame_action scene, SimpleDrone* drone)
	{
		for (Action* act : scene)
			act->accept(this, drone);
	}

	void visit(std::vector<frame_action> scene, SimpleDrone* drone)
	{
		for (frame_action frame : scene)
			this->visit(frame, drone);
	}
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

	void accept(SceneVisitor* visitor, SimpleDrone* drone)
	{
		visitor->visit(this, drone);
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

	void accept(SceneVisitor* visitor, SimpleDrone* drone)
	{
		visitor->visit(this, drone);
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

	void accept(SceneVisitor* visitor, SimpleDrone* drone)
	{
		visitor->visit(this, drone);
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