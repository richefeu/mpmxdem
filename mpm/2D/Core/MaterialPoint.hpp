#ifndef MATERIALPOINT_HPP
#define MATERIALPOINT_HPP

#include <vector>

#include "PBC3D.hpp"
#include "mat4.hpp"
#include "vec2.hpp"

struct ConstitutiveModel;

struct MaterialPoint {
  int nb;          // Number of the Material Point
  int groupNb;     // Group Number
  double mass;     // Mass
  double size;     // size of the sides of squared MP
  double vol0;     // Initial volume
  double vol;      // Current volume
  double density;  // Density
  vec2r pos;       // Position
  vec2r vel;       // Raw velocity (not smoothed)

  double securDist;  // Security distance for contact detection (it is updated as a function of MP velocity)
  vec2r f;           // Force
  vec2r q;           // Momentum (mass times velocity)

  mat4r strain;         // Total strain
  mat4r plasticStrain;  // Plastic Strain
  mat4r deltaStrain;    // Increment of strain (it is re-computed by ConstitutiveModel)

  mat4r stress;         // Total stress
  mat4r plasticStress;  // Plastic Stress
  double sigma3;        // third principal stress

  double N[16];     // Value of shape function according to the position of the Material Point
  vec2r gradN[16];  // Gradient of shape function according to the position of the Material Point
  int e;            // Identify the element to which the point belongs
  mat4r F;          // Deformation gradient matrix
  mat4r velGrad;    // Gradient of velocity (required eg. for computation of F)

  vec2r corner[4];  // Four corners according to F (positions expressed in the global frame)
  int splitCount;   // Generation number caused by successive to splits

  vec2r prev_pos;  // Position at the previous time step
  mat4r prev_F;    // Deformation gradient at the previous step

  bool plastic;                          // checks if the point was plastified
  vec2r contactf;                        // resultant force due to contacts only
  bool isDoubleScale;                    // use of numerically homogeneized law if true
  ConstitutiveModel* constitutiveModel;  // Pointer to the constitutive model
  PBC3Dbox* PBC;  // Pointer to a periodic 3D-DEM system (in case of 'homogeneised numerical law')

  // Ctor
  MaterialPoint(int Group = 0, double Vol = 0.0, double Rho = 0.0, ConstitutiveModel* CM = nullptr);

  // Some useful methods
  void updateCornersFromF();
};

#endif /* end of include guard: MATERIALPOINT_HPP */
