#pragma once

// Implementing my own matrices. Partly as a refresher and partly because I really don't need a lot of
// functionality so BLAS is probably overkill.

// Just 2D matrices for now...



template <size_t I, size_t J>
class matrix
{
public:

	float vals[I][J];
	size_t X = I;
	size_t Y = J;

	void setvals(std::vector<float> init_vals)
	{
		float* f = (float*)this->vals;

		for (float val : init_vals)
		{
			*f = val;
			f++;
		}
	}

	void displayvals()
	{
		for (int i = 0; i < I; i++)
		{
			for (int j = 0; j < J; j++)
				std::cout << this->vals[i][j] << " ";
			std::cout << "\n";
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

};

template <size_t I, size_t J, size_t K>
matrix <I, K> operator*(matrix<I, J> lhs, matrix<J, K> rhs)
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
			for (int j = 0; j < J; j++)
				out.vals[i][j] += lhs.vals[i][j] * rhs.vals[j][k];

	return out;
}