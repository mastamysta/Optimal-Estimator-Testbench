#include <iostream>
#include <vector>

#include "matrix.h"
#include "environmental_model.h"

#define RUN_TESTS

#ifdef RUN_TESTS

void matrix_tests()
{
	matrix<3, 3> A = { 1, 0, 2, 1, -1, 2, 0, 3, 3 };
	matrix<3, 1> B = { 1, 0, 0 };

	std::cout << A;
	std::cout << B;

	std::cout << (A * B); // should give [1, 0, 2]T

	std::cout << A.transpose();
}

#endif

int main()
{

#ifdef RUN_TESTS
	matrix_tests();
#endif

	SimpleDrone drone_model;

	std::cout << drone_model.get_nose_direction();

	// Okay I know I *said* I don't think in radians, but 180 degs is obvious.
	auto pi_about_x = useful_matrices::rotate_about_x(180);

	drone_model.transform_over(pi_about_x, 1);

	std::cout << drone_model.get_nose_direction();

	return 0;
}
