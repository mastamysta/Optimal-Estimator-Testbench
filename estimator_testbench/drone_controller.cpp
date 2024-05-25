#include "drone_controller.h"


void set_rotational_vibration(float amplitude, float variance)
{

}

void set_planar_vibration(float amplitude, float variance)
{

}

void perform_actions(std::vector<frame_action> scene, SimpleDrone* drone)
{
	for (frame_action frame : scene)
		for (Action* act : frame)
			act->doit(drone);
}

void run_simulation()
{

}
