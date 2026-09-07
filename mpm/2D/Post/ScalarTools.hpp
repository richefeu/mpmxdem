#pragma once

#include "mat4.hpp"
#include "mat9.hpp"

//
// The handful of invariants shared by the post-processing actions that build
// a mu(I) rheology, whether the stress comes from a DEM cell (DEMScalars) or
// from the macroscopic state of a Material Point (Scalars).
//
namespace ScalarTools {

// sqrt(J2) = sqrt(0.5 dev(S):dev(S)) of a 3x3 tensor.
// Unaffected by the sign convention of S, since dev(-S) = -dev(S).
double sqrtJ2(const mat9r &S);

// Shear rate sqrt(2 dev(D):dev(D)), with D the symmetric part of the velocity
// gradient L. For a simple shear of rate g this returns g.
double shearRate(const mat9r &L);

// Same, from the 2D velocity gradient of the MPM. The plane-strain assumption
// used throughout the coupling gives L_zz = 0.
double shearRate2D(const mat4r &L);

// Rebuilds the full 3x3 stress of a Material Point from what it stores: the
// in-plane 2x2 block, the out-of-plane normal component and the two
// out-of-plane shear components.
//
// For every classical constitutive model sxz and syz are zero -- the
// out-of-plane direction is principal, which is what plane strain means for
// the stress. They are only non-zero for CHCL_DEM, whose periodic cell is
// genuinely 3D. Passing them matters: they enter J2, hence tau and mu.
mat9r fullStress(const mat4r &stress2D, double szz, double sxz, double syz);

// Plane-strain projection of a 3x3 stress: the same tensor with its
// out-of-plane shears removed, ie the state a 2D plane-strain continuum would
// be able to reach.
//
// It leaves the trace, hence P, untouched, and only lowers sqrt(J2). Its use
// is to compare a DEM cell with a continuum on equal footing: a plane-strain
// continuum cannot develop sigma_xz or sigma_yz at all, so part of the gap
// between a micro mu and a macro one is structural rather than physical.
mat9r planeStrainProjection(const mat9r &S);

// Equivalent shear strain accumulated by a Material Point, read off its
// deformation gradient F.
//
// It is the Hencky (logarithmic) measure: with B = F F^T the left
// Cauchy-Green tensor and e = 0.5 ln(B) the Hencky strain,
//
//     gamma = sqrt(2 dev(e):dev(e))
//
// normalised so that a simple shear of amplitude g returns g -- the same
// convention as shearRate(), of which this is the time-integrated
// counterpart for a monotonic path. Plane strain gives F_zz = 1, hence a
// zero out-of-plane Hencky component.
//
// What it is NOT: the path length int gamma_dot dt. F only knows the current
// shape of the point, not how it got there, so a shear followed by the
// reverse shear returns zero. The two agree while the loading is monotonic,
// which is the case of a collapsing column but not of an arbitrary path. Note
// also that the logarithm compresses large values.
double equivalentShearStrain(const mat4r &F);

// Mobilised friction, read off the extreme principal stresses of a 3x3 stress
// tensor S given with COMPRESSION POSITIVE:
//
//     sinPhi = (S1 - S3) / (S1 + S3)
//
// with S1 the major and S3 the minor principal stress. This is the Mohr-circle
// ratio, so for a Mohr-Coulomb material at yield it returns sin(phi) exactly,
// and tan(phi) follows as sinPhi / sqrt(1 - sinPhi^2).
//
// It differs from mu = sqrt(J2)/P, which is what the mu(I) rheology uses
// (Jop, Forterre & Pouliquen 2006 define mu = |tau|/P with |tau| = sqrt(J2)).
// The two answer different questions: sqrt(J2)/P weighs the three principal
// stresses, hence depends on the intermediate one, while this ratio ignores
// it. Reporting both lets a Mohr-Coulomb run be read under either convention.
//
// Returns 0 when S1 + S3 <= 0, ie under tension.
double mobilisedSinPhi(const mat9r &S);

// tan(phi) from the above. Returns 0 if sinPhi is not in [0, 1[.
double sinPhiToTanPhi(double sinPhi);

} // namespace ScalarTools
