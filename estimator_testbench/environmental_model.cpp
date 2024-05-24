#include <vector>
#include <math.h>
#include <tgmath.h>

#include "matrix.h"
#include "environmental_model.h"

// Normalise a vector
template<size_t I>
static matrix<I, 1> normalise_3d(matrix<I, 1> in)
{
	// First grab magnitude

	float magnitude = 0;

	for (int i = 0; i < I; i++)
	{
		magnitude += pow(in.vals[i][0], 2);
	}

	magnitude = sqrtf(magnitude);

	return in / magnitude;

}

// Apply some linear transformation to our rotation state.
// Shears & scales won't make much sense in the context of translating 3 dimensional rotation
// space, so generally these will just be pure (unitary) rotations.
void SimpleDrone::updateState(matrix<3, 3> transformation)
{
	this->nose_direction = normalise_3d(transformation * this->nose_direction);
	this->right_direction = normalise_3d(transformation * this->right_direction);

	this->delta++;
}

matrix<3, 1> SimpleDrone::getPureAccelReadings()
{
	// Get angle between +X (forward direction of drone) and Z axis:
	//	a . b = |a| . |b| . cos ( theta )
	// vectors are normalized, so magnitudes are all 1
	// Projection onto span of Z transpose is equivalent to dot product.
	matrix<3, 1> Z = { 0, 0, 1 };

	auto intermediate = (Z.transpose() * this->nose_direction);

	auto theta_forward = acos(intermediate.vals[0][0]);
	auto theta_right = acos((Z.transpose() * this->right_direction).vals[1][1]);
	
	matrix<3, 1> ret = { 0, 0, 0 };

	return ret;
}

matrix<3, 1> SimpleDrone::getPureGyroReadings()
{
	matrix<3, 1> ret = {};

	return ret;
}