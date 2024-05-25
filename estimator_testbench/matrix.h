#pragma once

// Implementing my own matrices. Partly as a refresher and partly because I really don't need a lot of
// functionality so BLAS is probably overkill.

// Just 2D matrices for now...

template <size_t I, size_t J>
class matrix
{
public:
	float vals[I][J];

	void setvals(std::vector<float> init_vals)
	{
		float* f = (float*)this->vals;

		for (float val : init_vals)
		{
			*f = val;
			f++;
		}
	}

	matrix<J, I> transpose()
	{
		std::vector<float> tmp;

		for (int j = 0; j < J; j++)
			for (int i = 0; i < I; i++)
				tmp.push_back(this->vals[i][j]);

		matrix<J, I> ret;
		ret.setvals(tmp);
		return ret;
	}

	matrix<I, J> operator=(std::vector<float> var)
	{
		matrix<I, J> ret;
		return ret.setvals(var);
	}

};

template <size_t I, size_t J, size_t K>
matrix<I, K> operator*(matrix<I, J> lhs, matrix<J, K> rhs)
{
	//				 <- Y ->
	// [X x Y] -> /\[		]
	//			  X [		]
	//			  \/[		]

	// for Each column of rhs
	//		for each row in rhs
	//			for each

	matrix<I, K> out;

	for (int i = 0; i < I; i++)
		for (int k = 0; k < K; k++)
			out.vals[i][k] = 0;

	for (int i = 0; i < I; i++)
		for (int k = 0; k < K; k++)
			for (int j = 0; j < J; j++)
				out.vals[i][k] += lhs.vals[i][j] * rhs.vals[j][k];

	return out;
}

template <size_t I, size_t J>
matrix<I, J> operator*(float lhs, matrix<I, J> rhs)
{
	matrix<I, J> ret;

	for (int i = 0; i < I; i++)
		for (int j = 0; j < J; j++)
			ret.vals[i][j] = rhs.vals[i][j] * lhs;

	return ret;
}

template <size_t I, size_t J>
matrix<I, J> operator^(matrix<I, J> lhs, float rhs)
{
	matrix<I, J> ret;

	for (int i = 0; i < I; i++)
		for (int j = 0; j < J; j++)
			ret.vals[i][j] = pow(lhs.vals[i][j], rhs);

	return ret;
}

template <size_t I, size_t J>
matrix<I, J> operator+(matrix<I, J> lhs, matrix<I, J> rhs)
{
	matrix<I, J> ret;

	for (int i = 0; i < I; i++)
		for (int j = 0; j < J; j++)
			ret.vals[i][j] = lhs.vals[i][j] + rhs.vals[i][j];

	return ret;
}

// Scalar division, how fun!
template <size_t I, size_t J>
matrix<I, J> operator/(matrix<I, J> lhs, float rhs)
{
	matrix<I, J> out;

	for (int i = 0; i < I; i++)
			for (int j = 0; j < J; j++)
				out.vals[i][j] = lhs.vals[i][j] / rhs;

	return out;
}

// Make our module ostream-able. Useful for printing our matrices to stdout.
#include <iostream>
template<size_t I, size_t J>
std::ostream& operator<<(std::ostream& os, const matrix<I, J>& mat)
{
	for (int i = 0; i < I; i++)
	{
		for (int j = 0; j < J; j++)
			os << mat.vals[i][j] << " ";
		os << "\n";
	}

	os << "\n\n";

	return os;
}

namespace useful_matrices
{
	matrix<3, 3> rotate_about_x(float degrees);
	matrix<3, 3> rotate_about_y(float degrees);
	matrix<3, 3> rotate_about_z(float degrees);

	template <int I>
	matrix<I, I> identity()
	{
		matrix<I, I> ret;

		for (int i = 0; i < I; i++)
		{
			for (int j = 0; j < I; j++)
				ret.vals[i][j] = 0;

			ret.vals[i][i] = 1;
		}

		return ret;
	}
};