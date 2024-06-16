#include <vector>
#include <math.h>
#include <tgmath.h>

#include "matrix.h"
#include "environmental_model.h"

template <size_t I>
static float ipow(float f)
{
	float ret = 1;

	for (int i = 0; i < I; i++)
		ret *= f;

	return ret;
}

// Get 3D vector magitude
template <size_t I>
static float get_magnitude(matrix<I, 1> in)
{
	float magnitude = 0;

	// Profiling showed a bottleneck on pow, due to use of logarithms to resolve
	// assumed floating point exponent. Since we have a (known) integer exponent
	// we implement an integer power template.
	for (int i = 0; i < I; i++)
		//magnitude += pow(in.vals[i][0], 2);
		magnitude += ipow<2>(in.vals[i][0]);

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

	// To prevent us from accumulating error on the right_direction vector (which may not be perfectly
	// orthogonal to the nose_direction), we take the cross product of top and nose.
	//this->right_direction = vector_cross_product_3d(this->nose_direction, this->top_direction);

}

void SimpleDrone::nextDelta()
{
	x_rot = 0;
	y_rot = 0;
	z_rot = 0;

	this->delta++;
}

void SimpleDrone::rotate_about(float rads, matrix<3, 1> axis)
{
	// We need to find the matrix to rotate about the Y axis in the drone's frame.

	// a = current Y vector (right)

	// C	=	0	az	-ay
	//			-az	0	ax
	//			ay	-ax	0

	// R = I + Csin(theta) + C ^ 2 (1 - cos(theta))

	auto I = useful_matrices::identity<3>();

	auto a = axis.vals;

	matrix<3, 3> C = { 0,			a[2][0],	-a[1][0],
						-a[2][0],	0,			a[0][0],
						a[1][0],	-a[0][0],	0 };

	matrix<3, 3> R = I + (sin(rads) * C) + ((1 - cos(rads)) * (C ^ (size_t)2));

	updateState(R);
}

void SimpleDrone::pitch_rads(float rads)
{
	y_rot += rads;
	rotate_about(rads, this->right_direction);
}

void SimpleDrone::yaw_rads(float rads)
{
	z_rot += rads;
	rotate_about(rads, this->top_direction);
}

void SimpleDrone::roll_rads(float rads)
{
	x_rot += rads;
	rotate_about(rads, this->nose_direction);
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
matrix<3, 1> SimpleDrone::getPureGyroReadings()
{
	matrix<3, 1> ret = { x_rot, y_rot, z_rot };
	return ret;
}

matrix<3, 1> SimpleDrone::getAccelReadings()
{
	matrix<3, 1> noise = { accellerometer_distribution(generator),
							accellerometer_distribution(generator),
							accellerometer_distribution(generator) };

	return getPureAccelReadings() + noise;
}

matrix<3, 1> SimpleDrone::getGyroReadings()
{
	matrix<3, 1> noise = { gyroscope_distribution(generator),
							gyroscope_distribution(generator),
							gyroscope_distribution(generator) };

	return getPureGyroReadings() + noise;
}

// Return rotation in radians over the previous delta.
// TODO: Unfortunately this method doesn't work on it's own because it doesnt capture HOW the
// rotation occurred. E.g. did we rotate left or right to go through Pi Rads?
// Luckily this method does converge when the rotation for each delta is very small.
matrix<3, 1> SimpleDrone::getPureGyroReadings_deprecated()
{
	// To get rotations in X, Y & Z in the drone's frame, first we project the previous nose direction
	// onto the XY, XZ & ZY planes (in the drone's frame). We then measure the angle between these
	// projections and the current nose direction to get the yaw, pitch & roll respectively.

	// For convenience
	auto nose = this->nose_direction.vals;
	auto right = this->right_direction.vals;
	auto top = this->top_direction.vals;

	matrix<3, 3> xy_projection = { nose[0][0], right[0][0], 0,
									nose[1][0], right[1][0], 0,
									nose[2][0], right[2][0], 0 };

	matrix<3, 3> xz_projection = { nose[0][0], 0, top[0][0],
									nose[1][0], 0, top[1][0],
									nose[2][0], 0, top[2][0] };

	matrix<3, 3> yz_projection = { 0, right[0][0], top[0][0],
									0, right[1][0], top[1][0],
									0, right[2][0], top[2][0] };

	auto xy_prev_nose = (xy_projection * this->prev_nose_direction);
	auto xz_prev_right = (xz_projection * this->prev_right_direction);
	auto yz_prev_top = (yz_projection * this->prev_top_direction);

	// Find angle between projection of previous and current nose direction.
	// 
	// To find these we use the following identities:
	// 
	//			theta = acos( A . B / |A| . |B|)
	// 
	//			A . B = transpose(A) * B
	// 
	// A and B are normalised so this makes the calulation quite nice.

	matrix<3, 1> forward_in_drone_frame = { 1, 0, 0 };
	matrix<3, 1> right_in_drone_frame = { 0, 1, 0 };
	matrix<3, 1> up_in_drone_frame = { 0, 0 , 1 };

	auto bruh = (yz_prev_top.transpose() * this->top_direction).vals[0][0] - 2;

	auto pitch = acos((xz_prev_right.transpose() * this->right_direction).vals[0][0]);
	auto yaw = acos((xy_prev_nose.transpose() * this->nose_direction).vals[0][0]);
	auto roll = acos(bruh);

	matrix<3, 1> ret = { pitch, yaw, roll };

	return ret;
}

