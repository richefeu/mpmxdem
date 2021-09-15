#include "OneStep.hpp"

OneStep::~OneStep() {}

void OneStep::resetDEM(Obstacle* obst, vec2r gravity) {
  // ==== Delete computed resultants (force and moment) of rigid obstacles
  obst->force.reset();
  obst->mom = 0.0;
  obst->acc = gravity;
  obst->arot = 0.0;
}

void OneStep::moveDEM1(Obstacle* obst, double dt, bool activeNumericalDissipation) {
  // ==== Move the rigid obstacles according to their mode of driving  
  if (obst->isFree) {
    if (activeNumericalDissipation == false) {
      double dt_2 = 0.5 * dt;
      double dt2_2 = dt_2 * dt;
      obst->pos += obst->vel * dt + obst->acc * dt2_2;
      obst->vel += obst->acc * dt_2;
      obst->rot += obst->vrot * dt + obst->arot * dt2_2;
      obst->vrot += obst->arot * dt_2;
    }
  } else {  // velocity is imposed (rotations are supposed blocked)
    obst->pos += obst->vel * dt;
  }
}

void OneStep::moveDEM2(Obstacle* obst, double dt) {
  if (obst->isFree) {
    double dt_2 = 0.5 * dt;
    obst->acc = obst->force / obst->mass;
    obst->vel += obst->acc * dt_2;
    obst->arot = obst->mom / obst->I;
    obst->vrot += obst->arot * dt_2;
  }
}

vec2r OneStep::numericalDissipation(vec2r velMP, vec2r forceMP) {
  double halfSigned = 0.0;
  double product = forceMP * velMP;
  if (product >= 0.0)
    halfSigned = 0.5;
  else
    halfSigned = -0.5;

  // the value shouldnt be too high cuz it will freeze the material and
  // when its released the waves will reappear
  vec2r deltaF = -halfSigned * forceMP;
  forceMP += deltaF;

  return forceMP;
}
vec2r OneStep::numericalDissipation(vec2r velMP, vec2r forceMP, double cundall) {
  double cundallSigned = 0.0;
  double product = forceMP * velMP;
  if (product >= 0.0)
    cundallSigned = 0.5;
  else
    cundallSigned = -0.5;

  // the value shouldnt be too high cuz it will freeze the material and
  // when its released the waves will reappear
  vec2r deltaF = -cundallSigned * forceMP;
  forceMP += deltaF;

  return forceMP;
}
