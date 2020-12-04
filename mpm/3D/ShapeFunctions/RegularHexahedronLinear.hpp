#ifndef REGULARHEXAHEDRONLINEAR_HPP_5D5F2006
#define REGULARHEXAHEDRONLINEAR_HPP_5D5F2006

#include "../Core/MPMbox.hpp"
#include "ShapeFunction.hpp"
struct RegularHexahedronLinear : public ShapeFunction
{
	void computeInterpolationValues (MPMbox & MPM, size_t p);
};

#endif /* end of include guard: REGULARHEXAHEDRONLINEAR_HPP_5D5F2006 */
