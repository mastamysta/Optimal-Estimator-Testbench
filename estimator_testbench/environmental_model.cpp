#include <vector>
#include <math.h>
#include <tgmath.h>

#include "matrix.h"
#include "environmental_model.h"

// Get 3D vector magitude
template <size_t I>
static float get_magnitude(matrix<I, 1> in)
{
	float magnitude = 0;

	for (int i = 0; i < I; i++)
		magnitude += pow(in.vals[i][0], 2);

	return sqrtf(magnitude);
}

// Normalise a 3D vector
template<size_t I>
static matrix<I, 1> normalise_3d(matrix<I, 1> in)
{
	return in / get_magnitude(in);
}

static matrix<3, 1> vector_cross_product_3d(matrix<3, 1> lhs, matrix<3, 1> rhs)
{
	matrix<3, 1> ret = { lhs.vals[1][0] * rhs.vals[2][0] - lhs.vals[2][0] * rhs.vals[1][0],
						 lhs.vals[2][0] * rhs.vals[0][0] - lhs.vals[0][0] * rhs.vals[2][0],
						 lhs.vals[0][0] * rhs.vals[1][0] - lhs.vals[1][0] * rhs.vals[0][0] };
	return ret;
}


// Apply some linear transformation to our rotation state.
// Shears & scales won't make much sense in the context of translating 3 dimensional rotation
// space, so generally these will just be pure (unitary) rotations.
void SimpleDrone::updateState(matrix<3, 3> transformation)
{
	this->prev_nose_direction = this->nose_direction;
	this->prev_right_direction = this->prev_right_direction;
	this->prev_top_direction = this->top_direction;

	this->nose_direction = normalise_3d(transformation * this->nose_direction);
	this->right_direction = normalise_3d(transformation * this->right_direction);

	// Find perpendicular to nose and right vector - this is the direction of the top of the drone.
	this->top_direction = vector_cross_product_3d(this->nose_direction, this->right_direction);

	this->delta++;
}

// Return accelleration in Gs under current state.
matrix<3, 1> SimpleDrone::getPureAccelReadings()
{
	// Without any external accelleration, the Z components of the forward, right & upward vectors
	// (in the drone's frame) are the reaction forces measured due to locality. 
	// This works nicely because gravity is 1g and the nose and right vectors are normalised.
	matrix<3, 1> ret = { this->nose_direction.vals[2][0], 
							this->right_direction.vals[2][0], 
							this->top_direction.vals[2][0] };

	return ret;
}

// Return rotation in radians over the previous delta.
// TODO: Unfortunately this method doesn't work on it's own because it doesnt capture HOW the
// rotation occurred. E.g. did we rotate left or right to go through Pi Rads?
// Luckily this method does converge when the rotation for each delta is very small.
matrix<3, 1> SimpleDrone::getPureGyroReadings()
{
	// Find rotations (in the drone's frame) in X, Y, Z over the previous delta.
	// 
	// To find these we use the following identities:
	// 
	//			theta = acos( A . B / |A| . |B|)
	// 
	//			A . B = transpose(A) * B
	// 
	// A and B are normalised so this makes the calulation quite nice.

	auto x_rotation = acos((this->nose_direction.transpose() * this->prev_nose_direction).vals[0][0]);
	auto y_rotation = acos((this->right_direction.transpose() * this->prev_right_direction).vals[0][0]);
	auto z_rotation = acos((this->top_direction.transpose() * this->prev_top_direction).vals[0][0]);

	matrix<3, 1> ret = { x_rotation, y_rotation, z_rotation };

	return ret;
}


// Get angle between +X (forward direction of drone) and Z axis:
//	a . b = |a| . |b| . cos ( theta )
// vectors are normalized, so magnitudes are all 1
// Projection onto span of Z transpose is equivalent to dot product.
//matrix<3, 1> Z = { 0, 0, 1 };
//auto theta_forward = acos((Z.transpose() * this->nose_direction).vals[0][0]);
//auto theta_right = acos((Z.transpose() * this->right_direction).vals[0][0]);
//auto mag = get_magnitude(this->nose_direction);
