#ifndef UPDATESTRESSFIRST_HPP_64744727
#define UPDATESTRESSFIRST_HPP_64744727

#include "OneStep.hpp"
struct MPMbox;

struct UpdateStressFirst : public OneStep {
  virtual int advanceOneStep(MPMbox& MPM);
};

#endif /* end of include guard: UPDATESTRESSFIRST_HPP_64744727 */
