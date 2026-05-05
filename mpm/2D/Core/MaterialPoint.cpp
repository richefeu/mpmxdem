#include "MaterialPoint.hpp"

// Constructs a MaterialPoint with specified group number, size, and density.
//
// Initializes a MaterialPoint object with the given group number, size, and density.
// Sets the initial values for mass, volume, position, velocity, and other material properties.
// Initializes the constitutive model and related parameters.
//
// Group is the identifier-number for the material point.
// Size is the length of the sides of the squared material point.
// Rho is the initial density of the material point.
// CM Point to the constitutive model associated with the material point.
//
MaterialPoint::MaterialPoint(int Group, double Size, double Rho, ConstitutiveModel *CM)
    : groupNb(Group), size(Size), density(Rho), constitutiveModel(CM) {

  vol0   = size * size;
  vol    = vol0;
  mass   = vol0 * density;
  F      = mat4r::unit();
  prev_F = F;

  for (int i = 0; i < 16; i++) { N[i] = 0.0; }

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
  // FIXME: it assumes that the reference shape is an axis aligned rectangle
  // -> faire une fonction updateCornersFromFincrement

  double halfSizeMP = 0.5 * size;

  corner[0] = pos + F * vec2r(-halfSizeMP, -halfSizeMP);
  corner[1] = pos + F * vec2r(halfSizeMP, -halfSizeMP);
  corner[2] = pos + F * vec2r(halfSizeMP, halfSizeMP);
  corner[3] = pos + F * vec2r(-halfSizeMP, halfSizeMP);
}
