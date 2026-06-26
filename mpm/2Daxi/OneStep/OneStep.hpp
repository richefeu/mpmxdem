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

  virtual ~OneStep();  // Dtor
};
