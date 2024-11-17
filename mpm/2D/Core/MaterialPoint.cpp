#include "MaterialPoint.hpp"

/**
 * @brief Constructs a MaterialPoint with specified group number, size, and density.
 *
 * Initializes a MaterialPoint object with the given group number, size, and density.
 * Sets the initial values for mass, volume, position, velocity, and other material properties.
 * Initializes the constitutive model and related parameters.
 *
 * @param Group Group number for the material point.
 * @param Size Size of the sides of the squared material point.
 * @param Rho Initial density of the material point.
 * @param CM Pointer to the constitutive model associated with the material point.
 */
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
      outOfPlaneEp(0.0),
      deltaStrain(),
      stress(),
      outOfPlaneStress(0.0),
      hardeningForce(0.0),
      e(0),
      F(),
      velGrad(),
      splitCount(1),
      prev_pos(),
      plastic(false),
      contactf(),
      constitutiveModel(CM),
      isDoubleScale(false),
      PBC(nullptr),
      isTracked(false) {
  vol0 = vol = size * size;
  mass = vol0 * density;
  F = mat4r::unit();
  prev_F = F;
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

void MaterialPoint::updateCornersFromF() 
{
  // TODO: This is wrong if it's to be used in move_MP.cpp
  // FIXME: it assumes that the reference shape is an axis aligned rectangle
  // -> faire une fonction updateCornersFromFincrement

  double halfSizeMP = 0.5 * size;
  corner[0] = pos + F * vec2r(-halfSizeMP, -halfSizeMP);
  corner[1] = pos + F * vec2r(halfSizeMP, -halfSizeMP);
  corner[2] = pos + F * vec2r(halfSizeMP, halfSizeMP);
  corner[3] = pos + F * vec2r(-halfSizeMP, halfSizeMP);
}
