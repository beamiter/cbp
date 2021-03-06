// $Id$
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-16 Bradley M. Bell

CppAD is distributed under multiple licenses. This distribution is under
the terms of the
                    Eclipse Public License Version 1.0.

A copy of this license is included in the COPYING file of this distribution.
Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
-------------------------------------------------------------------------- */

/*
$begin mat_sum_sq.cpp$$
$spell
	sq
$$

$section Sum of the Elements of the Square of a Matrix: Example and Test$$
$mindex mat_sum_sq$$


$code
$srcfile%speed/example/mat_sum_sq.cpp%0%// BEGIN C++%// END C++%1%$$
$$

$end
*/
// BEGIN C++
# include <vector>
# include <cstddef>
# include <cppad/utility/vector.hpp>
# include <cppad/speed/mat_sum_sq.hpp>

bool mat_sum_sq()
{	bool   ok = true;
	double x_00, x_01, x_10, x_11, check;

	// dimension of the matrices x, y, and the result z
	size_t n = 2;
	CppAD::vector<double> x(n * n), y(n * n), z(1);

	// x = [ 1 2 ; 3 4 ]
	x[0] = x_00 = 1.;
	x[1] = x_01 = 2.;
	x[2] = x_10 = 3.;
	x[3] = x_11 = 4.;

	// compute y = x * x and z = sum of elements in y
	CppAD::mat_sum_sq(n, x, y, z);

	// check y_00
	check = x_00 * x_00 + x_01 * x_10;
	ok   &= (check == y[0]);

	// check y_01
	check = x_00 * x_01 + x_01 * x_11;
	ok   &= (check == y[1]);

	// check y_10
	check = x_10 * x_00 + x_11 * x_10;
	ok   &= (check == y[2]);

	// check y_11
	check = x_10 * x_01 + x_11 * x_11;
	ok   &= (check == y[3]);

	// check z
	check = y[0] + y[1] + y[2] + y[3];
	ok   &= (check == z[0]);

	return ok;
}
// END C++
