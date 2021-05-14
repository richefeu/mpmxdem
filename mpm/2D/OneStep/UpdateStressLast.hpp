#ifndef UPDATESTRESSLAST_HPP
#define UPDATESTRESSLAST_HPP

#include "OneStep.hpp"
struct MPMbox;

struct UpdateStressLast : public OneStep {
  virtual std::string getRegistrationName();
  virtual int advanceOneStep(MPMbox& MPM);
};

#endif /* end of include guard: UPDATESTRESSLAST_HPP */
