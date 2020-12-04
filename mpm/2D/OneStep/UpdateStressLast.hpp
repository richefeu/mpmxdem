#ifndef UPDATESTRESSLAST_HPP_64744727
#define UPDATESTRESSLAST_HPP_64744727

#include "OneStep.hpp"
struct MPMbox;

struct UpdateStressLast : public OneStep {
  virtual int advanceOneStep(MPMbox& MPM);
};

#endif /* end of include guard: UPDATESTRESSLAST_HPP_64744727 */
