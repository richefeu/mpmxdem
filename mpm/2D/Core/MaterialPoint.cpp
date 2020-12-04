#include "MaterialPoint.hpp"

MaterialPoint::MaterialPoint(int Group, double Size, double Rho, ConstitutiveModel* CM)
    : groupNb(Group),
      mass(0.0),
      size(Size),
      vol0(0.0),
      vol(0.0),
      density(Rho),
      pos(),
      vel(),
      securDist(0.0),
      f(),
      q(),
      strain(),
      plasticStrain(),
      deltaStrain(),
      stress(),
      e(0),
      F(),
      velGrad(),
      splitCount(1),
      prev_pos(),
      plastic(false),
      any(0),
      contactf(),
      constitutiveModel(CM) {
  vol0 = vol = size * size;
  mass = vol0 * density;
  F = mat4::unit();
  Fincrement = mat4::zero();
  for (int i = 0; i < 16; i++) N[i] = 0.0;

  gradN[0].reset();
  gradN[1].reset();
  gradN[2].reset();
  gradN[3].reset();

  corner[0].reset();
  corner[1].reset();
  corner[2].reset();
  corner[3].reset();
}

void MaterialPoint::updateCornersFromF() {
  // TODO: This is wrong if it's to be used in move_MP.cpp
  double halfSizeMP = 0.5 * size;

  corner[0] = pos + F * vec2r(-halfSizeMP, -halfSizeMP);
  corner[1] = pos + F * vec2r(halfSizeMP, -halfSizeMP);
  corner[2] = pos + F * vec2r(halfSizeMP, halfSizeMP);
  corner[3] = pos + F * vec2r(-halfSizeMP, halfSizeMP);
}
