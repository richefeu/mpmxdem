#include "OneStep.hpp"

OneStep::~OneStep() {}

void OneStep::resetDEM(Obstacle* obst, vec2r gravity) {
  // ==== Delete computed resultants (force and moment) of rigid obstacles
  obst->force.reset();
  obst->mom = 0.0;
  obst->acc = gravity;
  obst->arot = 0.0;
}

void OneStep::moveDEM1(Obstacle* obst, double dt) {
  // ==== Move the rigid obstacles according to their mode of driving
  if (obst->isFree) {
    double dt_2 = 0.5 * dt;
    double dt2_2 = dt_2 * dt;
    obst->pos += obst->vel * dt + obst->acc * dt2_2;
    obst->vel += obst->acc * dt_2;
    obst->rot += obst->vrot * dt + obst->arot * dt2_2;
    obst->vrot += obst->arot * dt_2;
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

vec2r OneStep::numericalDissipation(vec2r velMP, vec2r forceMP, double viscous_coefficient) {
  // Al-Kafaji PhD eq 4.131
  vec2r vecSigned;
  vecSigned.x = (forceMP.x * velMP.x > 0.0) ? 1.0 - viscous_coefficient : 1.0 + viscous_coefficient;
  vecSigned.y = (forceMP.y * velMP.y > 0.0) ? 1.0 - viscous_coefficient : 1.0 + viscous_coefficient;
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
