#include <iostream>
#include <vector>


#include "matrix.h"

#ifdef RUN_TESTS

void matrix_tests()
{
	matrix<3, 3> A;
	matrix<3, 1> B;

	A.setvals({ 1, 0, 2, 1, -1, 2, 0, 3, 3 });
	B.setvals({ 1, 0, 0 });

	A.displayvals();

	std::cout << "\n\n";

	B.displayvals();

	std::cout << "\n\n";

	(A * B).displayvals();

	std::cout << "\n\n";

	A.transpose().displayvals();
}

#endif

int main()
{

#ifdef RUN_TESTS
	matrix_tests();
#endif

	return 0;
}
