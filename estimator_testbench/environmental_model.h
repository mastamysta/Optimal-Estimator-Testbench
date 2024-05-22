#pragma once

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
	// Nose direction implicitly captures rotation of drone.
	matrix<3, 1> nose_direction = { 1, 0, 0 };

	// It is helpful to keep track of the current delta, to manage the evolution of the system.
	size_t delta = 0;

	void updateState(matrix<3, 3> transformation);

public:

	matrix<3, 1> get_nose_direction() { return nose_direction; }

	void transform_over(matrix<3, 3> transformation, size_t deltas);

};