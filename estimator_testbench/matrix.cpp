#pragma once

#include <iostream>
#include <vector>

#include "matrix.h"

#include <math.h>
#define PI 3.14159265

// Just a dumb helper class to grab well-known matrices such as pure unitary rotations
// Unfortunately C++ trig functions have side effects (errno), so no constexpr for us.
// This is particularly sad because trig will absolutely rinse the CPU.
// My rotations are in degrees because I don't think in radians.
namespace useful_matrices
{
	matrix<3, 3> rotate_about_x(float degrees)
	{
		float c = cos(degrees * PI / 180);
		float s = sin(degrees * PI / 180);
		matrix<3, 3> ret = { 1, 0, 0, 0, c, -s, 0, s, c };

		return ret;
	}

	matrix<3, 3> rotate_about_y(float degrees)
	{
		float c = cos(degrees * PI / 180);
		float s = sin(degrees * PI / 180);
		matrix<3, 3> ret = { c, 0, s, 0, 1, 0, -s, 0, c };

		return ret;
	}

	matrix<3, 3> rotate_about_z(float degrees)
	{
		float c = cos(degrees * PI / 180);
		float s = sin(degrees * PI / 180);
		matrix<3, 3> ret = { c, -s, 0, s, c, 0, 0, 0, 1 };

		return ret;
	}
};