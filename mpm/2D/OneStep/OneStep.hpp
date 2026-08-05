#pragma once

#include <fstream>
#include <string>

#include "Core/MaterialPoint.hpp"
#include "Obstacles/Obstacle.hpp"

class MPMbox;

struct OneStep {
  virtual std::string getRegistrationName() = 0;
  virtual int advanceOneStep(MPMbox& MPM) = 0;

  // the following methods are NOT virtual
  void resetDEM(Obstacle* obst, vec2r gravity);
  void moveDEM1(Obstacle* obst, double dt);
  void moveDEM2(Obstacle* obst, double dt);
  vec2r numericalDissipation(vec2r velMP, vec2r forceMP, double coefficient);

  // Steps shared by every integration scheme. They live here so that a feature
  // added to one of them is available to all three: the FLIP/PIC blending and
  // the double-scale dispatch used to exist in ModifiedLagrangian only, which
  // made 'enablePIC', the PICDissipation schedulers and the CHCL models
  // silently inoperative with the two others.
  void updateMPVelocity(MPMbox& MPM);
  void updateStrainAndStress(MPMbox& MPM);
  void updateDensityFromVolume(MPMbox& MPM);

  virtual ~OneStep();  // Dtor
};
