#ifndef MODIFIEDLAGRANGIAN_HPP
#define MODIFIEDLAGRANGIAN_HPP

#include "OneStep.hpp"
struct MPMbox;

struct ModifiedLagrangian : public OneStep {
  std::string getRegistrationName();
  int advanceOneStep(MPMbox& MPM);
};

#endif /* end of include guard: MODIFIEDLAGRANGIAN_HPP */
