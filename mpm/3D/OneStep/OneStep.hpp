#ifndef ONESTEP_HPP_64744727
#define ONESTEP_HPP_64744727

#include <fstream>
struct MPMbox;

struct OneStep
{
	MPMbox *box;

	virtual void plug(MPMbox *Box);

	virtual void advanceOneStep(MPMbox & MPM) = 0;

	virtual ~OneStep(); // Dtor
};

#endif /* end of include guard: ONESTEP_HPP_64744727 */
