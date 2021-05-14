#ifndef UPDATESTRESSFIRST_HPP
#define UPDATESTRESSFIRST_HPP

#include "OneStep.hpp"
struct MPMbox;

struct UpdateStressFirst : public OneStep {
  virtual std::string getRegistrationName();
  virtual int advanceOneStep(MPMbox& MPM);
};

#endif /* end of include guard: UPDATESTRESSFIRST_HPP */
