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

}
