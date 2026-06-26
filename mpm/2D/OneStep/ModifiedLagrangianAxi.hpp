#pragma once

// 
// Header file for the ModifiedLagrangianAxi OneStep implementation.
// 
// This file contains the declaration of the ModifiedLagrangianAxi class, which
// is a concrete implementation of the OneStep interface.
// 
// The ModifiedLagrangianAxi class is a OneStep implementation that uses the modified
// Lagrangian method to advance the Material Points in time.
// 

#include "OneStep.hpp"
class MPMbox;

struct ModifiedLagrangianAxi : public OneStep {
  std::string getRegistrationName();
  int advanceOneStep(MPMbox& MPM);
};
