#pragma once

#include "OneStep.hpp"
class MPMbox;

struct UpdateStressLast : public OneStep {
  std::string getRegistrationName();
  int advanceOneStep(MPMbox& MPM);
};
