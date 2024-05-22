#include <vector>

#include "matrix.h"
#include "environmental_model.h"


// Apply some linear transformation to our rotation state.
// Shears & scales won't make much sense in the context of translating 3 dimensional rotation
// space, so generally these will just be pure (unitary) rotations.
void SimpleDrone::updateState(matrix<3, 3> transformation)
{
	//std::cout << transformation * this->nose_direction;

	this->nose_direction = transformation * this->nose_direction;
	this->delta++;
}

// Apply some transformation over a number of deltas by dividing each element of the transformation
// matrix by the number of deltas it is to be applied over. This is considered a linear distribution
// of the transformation because the eigenvectors of each (sub)transformation are identical.
void SimpleDrone::transform_over(matrix<3, 3> transformation, size_t deltas)
{
	auto subtransformation = transformation / deltas;

	for (int i = 0; i < deltas; i++)
		updateState(subtransformation);
}