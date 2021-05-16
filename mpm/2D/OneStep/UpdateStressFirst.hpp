#ifndef UPDATESTRESSFIRST_HPP
#define UPDATESTRESSFIRST_HPP

#include "OneStep.hpp"
class MPMbox;

struct UpdateStressFirst : public OneStep {
  std::string getRegistrationName();
  int advanceOneStep(MPMbox& MPM);
};

#endif /* end of include guard: UPDATESTRESSFIRST_HPP */
