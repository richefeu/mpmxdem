#ifndef LINREG_HPP
#define LINREG_HPP

#include <vector>
#include <iostream>
#include <cmath>

class linreg {
	
	/// @brief Make a linear regression of a set of (x, y) pairs
	/// @return Returns a, b so that the line ax+b is the linear regression of data x and y.
	///         r is the coefficient of correlation.
    public : 
    double slope=0;
    double orig=0;
    double corr=0;
   
	void run(std::vector<double> &x, std::vector<double> &y)
	{
		if (x.size() != y.size()) {
			std::cerr << "@linreg, x and y must have the same size" << std::endl;
			return;
		}
		int N = static_cast<int>(x.size());
		if (N == 0) return;
		double invN = 1.0f / (double)N;

		double xmean = 0.0;
		double ymean = 0.0;
		for (int i = 0 ; i < N ; i++) {
			xmean += x[i];
			ymean += y[i];
		}
		xmean *= invN;
		ymean *= invN;

		double Sx = 0.0;
		double Sy = 0.0;
		double Sxy = 0.0;
		for (int i = 0 ; i < N ; i++) {
			double dx = x[i] - xmean;
			double dy = y[i] - ymean;
			Sx += dx * dx;
			Sy += dy * dy;
			Sxy += dx * dy;
		}

		slope = Sxy / Sx; // suppose that Sx cannot be zero
		orig = ymean - slope * xmean;
		if (Sy == 0.0) {
			corr = 1.0;
		}
		else {
			corr = Sxy / sqrt(Sx * Sy);
		}
	}
}; // end namespace

#endif /* end of include guard: LINREG_HPP */
