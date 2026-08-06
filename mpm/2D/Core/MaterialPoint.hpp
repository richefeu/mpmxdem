#pragma once

// 
// Defines the MaterialPoint structure and related includes.
// 
// This header file contains the definition of the MaterialPoint structure,
// which represents a material point in a simulation. It includes necessary
// libraries and forward declarations needed for the structure's implementation.
// 
// The MaterialPoint structure includes properties such as mass, size, volume,
// and group number, which are essential for simulations involving material
// points.
// 

#include <vector>

#include "PBC3D.hpp"
#include "mat4.hpp"
#include "vec2.hpp"

struct ConstitutiveModel;

//
// State that belongs to the constitutive model of a Material Point, and to it
// alone: nothing outside updateStrainAndStress reads or writes it.
//
// It sits in an array of MPMbox rather than in MaterialPoint so that the loops
// which walk the points at every step -- the transfers to and from the grid,
// which are the bulk of a time step -- do not drag it through the cache. It is
// reached with MPMbox::modelState(p).
//
struct MPModelState {
  // Viscous part of the total stress (KelvinVoigt only). Unlike the elastic
  // part, which is integrated increment by increment, this one is an
  // INSTANTANEOUS quantity: eta times the current strain rate. It is kept so
  // that the value of the previous step can be removed from 'stress' before the
  // new one is added -- otherwise the viscous term accumulates and behaves as an
  // extra stiffness eta/dt instead of a damper.
  // (NOT saved in the conf-files yet, see C3 in Doc/BUGS.md)
  mat4r viscousStress;
  double outOfPlaneViscousStress{0.0};

  double outOfPlaneEp{0.0};   // Out-of-plane plastic strain component (Sinfonietta)
  double hardeningForce{0.0}; // memory for hardening (Sinfonietta)
};

struct MaterialPoint {
  size_t nb{0};        // Number of the Material Point
  int groupNb{0};      // Group Number
  double mass{0.0};    // Mass (supposed constant)
  double size{0.0};    // size of the sides of squared MP
  double vol0{0.0};    // Initial volume
  double vol{0.0};     // Current volume
  double density{0.0}; // Density (can change because of volume changes)
  vec2r pos;           // Position
  vec2r vel;           // Raw velocity (not smoothed)

  double securDist{0.0}; // Security distance for contact detection (it is updated as a function of MP velocity)
  vec2r f;               // Force

  mat4r strain;             // Total strain
  mat4r plasticStrain;      // Plastic Strain
  mat4r deltaStrain;        // Increment of strain (it is computed in ConstitutiveModel for processing purpose)
                            // (FIXME: remove?)

  mat4r stress;                 // Total stress
  mat4r stressCorrection;       // Plastic Stress (REMARQUE à enlever ou renomer). C'est la correction plastic en
                                // fait. Ce truc avait été ajouté par Fabio.
  double outOfPlaneStress{0.0}; // Out-of-plane total stress component

  // The shape functions N and their gradients gradN used to live here, as
  // double N[16] and vec2r gradN[16], i.e. 384 bytes -- 43 % of this structure.
  // They now sit in two contiguous arrays of MPMbox, reachable through
  // MPMbox::N(p) and MPMbox::gradN(p): the array of Material Points shrinks by
  // as much, and the shape functions are read as a stream. See Doc/OPTIM.md.
  size_t e{0};     // Identify the element to which the point belongs
  mat4r F;         // Deformation gradient matrix
  mat4r velGrad;   // Gradient of velocity (required eg. for computation of F)

  // The four corners of the point used to be stored here, and refreshed at
  // every step from F. Nothing needed them: Polygon builds its contact frame
  // from F directly, and the viewer recomputes them into ProcessedDataMP.
  int splitCount{1}; // Generation number caused by successive to splits

  vec2r prev_pos; // Position at the previous time step

  bool plastic{false}; // checks if the point was plastified (TO BE REMOVED ?)
  vec2r contactf;      // resultant force due to contacts only

  ConstitutiveModel *constitutiveModel{nullptr}; // Pointer to the constitutive model
  bool isDoubleScale{false};                     // use of numerically homogeneized law if true
  PBC3Dbox *PBC{nullptr}; // Pointer to a periodic 3D-DEM system (in case of 'homogeneised numerical law')
  bool isTracked{false};  // if true, and if the MP isDoubleScale is true, save the DEM conf-file is separated folders

  // Ctor
  MaterialPoint(int Group = 0, double Size = 0.0, double Rho = 0.0, ConstitutiveModel *CM = nullptr);
};
