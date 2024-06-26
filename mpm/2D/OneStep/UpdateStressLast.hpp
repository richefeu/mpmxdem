#ifndef UPDATESTRESSLAST_HPP
#define UPDATESTRESSLAST_HPP

#include "OneStep.hpp"
class MPMbox;

struct UpdateStressLast : public OneStep {
  std::string getRegistrationName();
  int advanceOneStep(MPMbox& MPM);
};

#endif /* end of include guard: UPDATESTRESSLAST_HPP */
