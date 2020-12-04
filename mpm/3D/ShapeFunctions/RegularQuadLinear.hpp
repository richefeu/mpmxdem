#ifndef REGULARQUADLINEAR_HPP_5D5F2006
#define REGULARQUADLINEAR_HPP_5D5F2006

#include "../Core/MPMbox.hpp"
#include "ShapeFunction.hpp"
struct RegularQuadLinear : public ShapeFunction
{
	void computeInterpolationValues (MPMbox & MPM, size_t p);
};

#endif /* end of include guard: REGULARQUADLINEAR_HPP_5D5F2006 */
