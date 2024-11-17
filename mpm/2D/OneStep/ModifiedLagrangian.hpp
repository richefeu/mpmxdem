#ifndef MODIFIEDLAGRANGIAN_HPP
#define MODIFIEDLAGRANGIAN_HPP

/**
 * @file ModifiedLagrangian.hpp
 * @brief Header file for the ModifiedLagrangian OneStep implementation.
 *
 * This file contains the declaration of the ModifiedLagrangian class, which
 * is a concrete implementation of the OneStep interface.
 *
 * The ModifiedLagrangian class is a OneStep implementation that uses the modified
 * Lagrangian method to advance the Material Points in time.
 *
 */

#include "OneStep.hpp"
class MPMbox;

struct ModifiedLagrangian : public OneStep {
  std::string getRegistrationName();
  int advanceOneStep(MPMbox& MPM);
};



#endif /* end of include guard: MODIFIEDLAGRANGIAN_HPP */
