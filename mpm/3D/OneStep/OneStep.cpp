#include "OneStep.hpp"

OneStep::~OneStep() { }

void OneStep::plug(MPMbox *Box)
{
	box = Box;
}
