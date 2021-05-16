#ifndef ONESTEP_HPP
#define ONESTEP_HPP

#include <fstream>
#include <string>

#include "Core/MaterialPoint.hpp"
#include "Obstacles/Obstacle.hpp"

#include "DataTable.hpp" // util ??

class MPMbox;

struct OneStep {
  MPMbox* box;

  virtual void plug(MPMbox* Box);

  virtual std::string getRegistrationName() = 0;
  virtual int advanceOneStep(MPMbox& MPM) = 0;
  void resetDEM(Obstacle* obst, vec2r gravity);
  void moveDEM1(Obstacle* obst, double dt, bool activeNumericalDissipation);
  void moveDEM2(Obstacle* obst, double dt);
  vec2r numericalDissipation(vec2r velMP, vec2r forceMP);

  virtual ~OneStep();  // Dtor
};

#endif /* end of include guard: ONESTEP_HPP */
