#include "OneStep.hpp"

OneStep::~OneStep() {}

void OneStep::resetDEM(Obstacle *obst, vec2r gravity) {
  // ==== Delete computed resultants (force and moment) of rigid obstacles
  switch (obst->drive_mode) {
  case IMPOSE_FORCE:
    obst->force = obst->impForces[0] * obst->normal;
    obst->acc.reset();
    break;
  default:
    obst->force.reset();
    obst->acc = gravity;
    break;
  }
  obst->mom  = 0.0;
  obst->arot = 0.0;
}

void OneStep::moveDEM1(Obstacle *obst, double dt) {
  // ==== Move the rigid obstacles according to their mode of driving
  double dt_2  = 0.5 * dt;
  double dt2_2 = dt_2 * dt;
  switch (obst->drive_mode) {
  case IS_FREE:
    obst->pos += obst->vel * dt + obst->acc * dt2_2;
    obst->vel += obst->acc * dt_2;
    obst->rot += obst->vrot * dt + obst->arot * dt2_2;
    obst->vrot += obst->arot * dt_2;
    break;

  case IMPOSE_VELOCITY:
    obst->pos += obst->vel * dt;
    break;

  case IMPOSE_FORCE:
    obst->pos += obst->vel * dt + obst->acc * dt2_2;
    obst->vel += obst->acc * dt_2;
    break;

  default:
    break;
  }
}

void OneStep::moveDEM2(Obstacle *obst, double dt) {
  double dt_2 = 0.5 * dt;
  switch (obst->drive_mode) {
  case IS_FREE:
    obst->acc = obst->force / obst->mass;
    obst->vel += obst->acc * dt_2;
    obst->arot = obst->mom / obst->I;
    obst->vrot += obst->arot * dt_2;
    break;
  case IMPOSE_FORCE:
    obst->acc = obst->force / obst->mass - obst->damp * obst->vel;
    obst->vel += obst->acc * dt_2;
    break;
  default:
    break;
  }
}

vec2r OneStep::numericalDissipation(vec2r velMP, vec2r forceMP, double coefficient) {
  vec2r vecSigned;
  vecSigned.x = (forceMP.x * velMP.x > 0.0) ? (1.0 - coefficient) : (1.0 + coefficient);
  vecSigned.y = (forceMP.y * velMP.y > 0.0) ? (1.0 - coefficient) : (1.0 + coefficient);
  return vecSigned;
}

/*
vec2r OneStep::numericalDissipation(vec2r velMP, vec2r forceMP, double cundall) {
  double factor;
  double factorMinus = 1.0 - cundall;
  double factorPlus = 1.0 + cundall;
  factor = (forceMP.x * velMP.x > 0.0) ? factorMinus : factorPlus;
  forceMP.x *= factor;
  factor = (forceMP.y * velMP.y > 0.0) ? factorMinus : factorPlus;
  std::cout << "cundall factor" << factor << std::endl;
  forceMP.y *= factor;
  return forceMP;
}
*/
