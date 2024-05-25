#pragma once

#include <random>

#include "matrix.h"


// Classes deriving from EnvironmentalModel will be used to store some state and produce some
// sensor readings for that state. EnvironmentalModels will introduce random noise to simulate real
// measurements.
// EnvironmentalModels will capture discrete states of arbitrary delta 1.
class EnvironmentalModel{};

// The SimpleDrone will produce gyroscopic and accellerometer readings.
// Accellerometer readings get noisier when the drone experiences vibrations.
// Gyroscopic readings simply accumulate error (over some integration).
// Sensor readings follow ordering X, Y, Z
class SimpleDrone : public EnvironmentalModel
{

private:

	std::default_random_engine generator;
	std::normal_distribution<float> accellerometer_distribution;
	std::normal_distribution<float> gyroscope_distribution;

	// Accellerometer noise parameters
	size_t accellerometer_noise_mean = 0;
	size_t accellerometer_noise_standard_dev = 2;

	// Gyroscope noise parameters
	float gyroscope_noise_mean = 0.1;
	float gyroscope_noise_standard_dev = 0.3;

	// Nose direction implicitly captures rotation of drone.
	matrix<3, 1> nose_direction = { 1, 0, 0 };
	matrix<3, 1> right_direction = { 0, 1, 0 };
	matrix<3, 1> top_direction = { 0, 0, 1 };

	matrix<3, 1> prev_nose_direction = { 1, 0, 0 };
	matrix<3, 1> prev_right_direction = { 0, 1, 0 };
	matrix<3, 1> prev_top_direction = { 0, 0, 1 };

	// Record rotation operations applied this delta
	float x_rot = 0, y_rot = 0, z_rot = 0;

	// It is helpful to keep track of the current delta, to manage the evolution of the system.
	size_t delta = 0;

	void updateState(matrix<3, 3> transformation);
	void rotate_about(float rads, matrix<3, 1> axis);

public:

	SimpleDrone()
	{
		accellerometer_distribution = std::normal_distribution<float>(
										accellerometer_noise_mean,
										accellerometer_noise_standard_dev);

		gyroscope_distribution = std::normal_distribution<float>(
									gyroscope_noise_mean,
									gyroscope_noise_standard_dev);
	}

	void nextDelta();

	void pitch_rads(float rads);
	void yaw_rads(float rads);
	void roll_rads(float rads);

	matrix<3, 1> get_nose_direction() { return nose_direction; }
	matrix<3, 1> get_right_direction() { return right_direction; }
	matrix<3, 1> get_top_direction() { return top_direction; }

	// Readings with no noise
	matrix<3, 1> getPureAccelReadings();
	matrix<3, 1> getPureGyroReadings();
	matrix<3, 1> getPureGyroReadings_deprecated();

	// Readings including noise
	matrix<3, 1> getAccelReadings();
	matrix<3, 1> getGyroReadings();

};