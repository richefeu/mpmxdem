#ifndef SHAPEFUNCTION_HPP_5D5F2006
#define SHAPEFUNCTION_HPP_5D5F2006

#include "../Core/MPMbox.hpp"
#include <cstdlib>
struct MPMbox;

struct ShapeFunction
{
	virtual ~ShapeFunction();

	// This function compute N and gradN of the MaterialPoint MPM.MP[p]
	virtual void computeInterpolationValues (MPMbox & MPM, size_t p) = 0;
};


#endif /* end of include guard: SHAPEFUNCTION_HPP_5D5F2006 */
