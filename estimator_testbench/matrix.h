#pragma once

// Implementing my own matrices. Partly as a refresher and partly because I really don't need a lot of
// functionality so BLAS is probably overkill.

// Just 2D matrices for now...


template <int... Is> // Base
struct matrix_wrapper
{
	using type = float;
};

template <int I, int... Is> // Specialise
struct matrix_wrapper<I, Is...>
{
	using type = matrix_wrapper<Is...>::type[I];
};

template <int... Is>
using matrix_blob = matrix_wrapper<Is...>::type;

template <int... Is>
class matrix
{
private:
	matrix_blob<Is...> v;
};