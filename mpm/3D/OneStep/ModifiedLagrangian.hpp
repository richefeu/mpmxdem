#ifndef MODIFIEDLAGRANGIAN_HPP_64744727
#define MODIFIEDLAGRANGIAN_HPP_64744727

#include "OneStep.hpp"
struct MPMbox;

struct ModifiedLagrangian: public OneStep
{
	virtual void advanceOneStep(MPMbox & MPM);
};

#endif /* end of include guard: MODIFIEDLAGRANGIAN_HPP_64744727 */
