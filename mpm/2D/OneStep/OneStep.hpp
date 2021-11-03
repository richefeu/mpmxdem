#ifndef ONESTEP_HPP
#define ONESTEP_HPP

#include <fstream>
#include <string>

#include "Core/MaterialPoint.hpp"
#include "Obstacles/Obstacle.hpp"

class MPMbox;

struct OneStep {
  virtual std::string getRegistrationName() = 0;
  virtual int advanceOneStep(MPMbox& MPM) = 0;
  void resetDEM(Obstacle* obst, vec2r gravity);
  void moveDEM1(Obstacle* obst, double dt);
  void moveDEM2(Obstacle* obst, double dt);
  vec2r numericalDissipation(vec2r velMP, vec2r forceMP, double viscousDissipation);
//  vec2r numericalDissipation(vec2r velMP, vec2r forceMP, double cundall);

  virtual ~OneStep();  // Dtor
};

#endif /* end of include guard: ONESTEP_HPP */
