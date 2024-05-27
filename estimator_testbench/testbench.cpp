#include <iostream>
#include <vector>

#include "matrix.h"
#include "environmental_model.h"
#include "drone_controller.h"

//#define RUN_TESTS

#ifdef RUN_TESTS

void matrix_tests()
{
	matrix<3, 3> A = { 1, 0, 2, 1, -1, 2, 0, 3, 3 };
	matrix<3, 1> B = { 1, 0, 0 };

	std::cout << A;
	std::cout << B;

	std::cout << (A * B); // should give [1, 1, 0]T

	std::cout << A.transpose();
}

void rotation_tests()
{
	std::cout << drone_model.get_nose_direction();

	drone_model.roll_rads(PI / 4);
	drone_model.yaw_rads(PI / 8);

	std::cout << "nose direction:\n";
	std::cout << drone_model.get_nose_direction();
	std::cout << "right direction:\n";
	std::cout << drone_model.get_right_direction();
	std::cout << "top direction:\n";
	std::cout << drone_model.get_top_direction();

	std::cout << drone_model.getPureAccelReadings();
	std::cout << drone_model.getPureGyroReadings();
}

#endif

#define PI 3.14159265

int main()
{

#ifdef RUN_TESTS
	matrix_tests();
	rotation_tests();
#endif

	SimpleDrone drone_model;

	// Build instruction chains for a set of commanded rotations over 100k deltas
	scene scene;
	delta frame;

	float total_rotation = 314;
	float yaw_deltas = 10000;
	float delta_rotation = total_rotation / yaw_deltas;

	YawAction *yaw = new YawAction(delta_rotation);
	RepeatAction r((Action*)yaw, yaw_deltas);

	frame.push_back((Action*)&r);
	scene.push_back(frame);

	DroneController dc(LOGGING_CSV);
	dc.set_logging_path(".\\dingus.csv");
	dc.run_simulation(drone_model, scene);

	return 0;
}
