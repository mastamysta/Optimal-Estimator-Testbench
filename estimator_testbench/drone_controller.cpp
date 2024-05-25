#include "drone_controller.h"




void set_rotational_vibration(float amplitude, float variance)
{

}

void set_planar_vibration(float amplitude, float variance)
{

}

void perform_actions(std::vector<frame_action> scene, SimpleDrone* drone)
{
	SceneVisitor visitor;

	visitor.visit(scene, drone);
}

void run_simulation()
{

}
