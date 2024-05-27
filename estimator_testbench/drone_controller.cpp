#include <string>
#include <fstream>

#include "drone_controller.h"

// Set parameters for gaussian noise added to the 'commanded' rotation of the drone.
// TODO: This might be better modelled by a noisy sinusoid, given it is meant to represent vibrations
// rather than white noise!
void DroneController::set_rotational_vibration(float amplitude, float variance)
{

}

// Not implemented -- drone does not support non-gravitational accelleration yet!
void DroneController::set_planar_vibration(float amplitude, float variance)
{

}

void DroneController::do_logging()
{
	switch (this->logging)
	{
	case LOGGING_CSV:
		log_data.push_back("Hello world!\n");
		return;

	case LOGGING_NONE: // FALLTHROUGH
	default:
		return;
	}
}

void DroneController::done()
{
	// Because file IO is slow AF, we try to keep the data in-process until the very end of the sim
	// and emit it all to file in one go.
	std::ofstream file(logging_path);

	if (!file.is_open())
	{
		std::cout << "Failed to open CSV and emit data.\n";
		return;
	}

	for (std::string line : log_data)
		file << line;

	file.close();

	std::cout << "Logging was successful: " << logging_path << ".\n";

}

void DroneController::perform_actions(scene scene, SimpleDrone& drone)
{
	for (delta frame : scene)
	{
		for (Action* act : frame)
		{
			act->doit(drone);
			// TODO: Add commands for vibrations
		}

		do_logging();
	}

	done();
}

void DroneController::run_simulation(SimpleDrone& drone, scene& scene)
{
	perform_actions(scene, drone);
}
