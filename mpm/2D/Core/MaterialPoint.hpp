#ifndef MATERIALPOINT_HPP
#define MATERIALPOINT_HPP

/**
 * @file MaterialPoint.hpp
 * @brief Defines the MaterialPoint structure and related includes.
 *
 * This header file contains the definition of the MaterialPoint structure,
 * which represents a material point in a simulation. It includes necessary
 * libraries and forward declarations needed for the structure's implementation.
 *
 * The MaterialPoint structure includes properties such as mass, size, volume,
 * and group number, which are essential for simulations involving material
 * points.
 *
 */


#include <vector>

#include "PBC3D.hpp"
#include "mat4.hpp"
#include "vec2.hpp"

struct ConstitutiveModel;

struct MaterialPoint {
  size_t nb;                // Number of the Material Point
  int groupNb;              // Group Number
  double mass;              // Mass (supposed constant)
  double size;              // size of the sides of squared MP
  double vol0;              // Initial volume
  double vol;               // Current volume
  double density;           // Density (can change because of volume changes)
  vec2r pos;                // Position
  vec2r vel;                // Raw velocity (not smoothed)

  double securDist;         // Security distance for contact detection (it is updated as a function of MP velocity)
  vec2r f;                  // Force
  vec2r q;                  // Momentum (mass times velocity)

  mat4r strain;             // Total strain
  mat4r plasticStrain;      // Plastic Strain 
  double outOfPlaneEp;      // Out-of-plane plastic strain component
  mat4r deltaStrain;        // Increment of strain (it is computed in ConstitutiveModel for processing purpose) (FIXME: remove?)

  mat4r stress;             // Total stress
  mat4r stressCorrection;   // Plastic Stress (REMARQUE à enlever ou renomer). C'est la correction plastic en
                            // fait. Ce truc avait été ajouté par Fabio.
  double outOfPlaneStress;  // Out-of-plane total stress component
  double hardeningForce;    // memory for hardening

  double N[16];             // Value of shape function according to the position of the Material Point
  vec2r gradN[16];          // Gradient of shape function according to the position of the Material Point
  size_t e;                 // Identify the element to which the point belongs
  mat4r F;                  // Deformation gradient matrix
  mat4r velGrad;            // Gradient of velocity (required eg. for computation of F)

  //vec2r refShape[4];
  // bool hasRefShape;
  vec2r corner[4];          // Four corners according to F (positions expressed in the global frame)
  int splitCount;           // Generation number caused by successive to splits

  vec2r prev_pos;           // Position at the previous time step
  mat4r prev_F;             // Deformation gradient at the previous step

  bool plastic;             // checks if the point was plastified (TO BE REMOVED)
  vec2r contactf;           // resultant force due to contacts only (TO BE REMOVED)

  ConstitutiveModel* constitutiveModel;  // Pointer to the constitutive model
  bool isDoubleScale;                    // use of numerically homogeneized law if true
  PBC3Dbox* PBC;   // Pointer to a periodic 3D-DEM system (in case of 'homogeneised numerical law')
  bool isTracked;  // if true, and if the MP isDoubleScale is true, save the DEM conf-file is separated folders

  // Ctor
  MaterialPoint(int Group = 0, double Size = 0.0, double Rho = 0.0, ConstitutiveModel* CM = nullptr);

  // Some useful method(s)
  void updateCornersFromF();
};

#endif /* end of include guard: MATERIALPOINT_HPP */
