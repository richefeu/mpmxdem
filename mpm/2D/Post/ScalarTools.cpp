#include "ScalarTools.hpp"

#include <cmath>

namespace ScalarTools {

double sqrtJ2(const mat9r &S) {
  const double p   = S.trace() / 3.0;
  const double dxx = S.xx - p;
  const double dyy = S.yy - p;
  const double dzz = S.zz - p;
  const double dxy = 0.5 * (S.xy + S.yx);
  const double dxz = 0.5 * (S.xz + S.zx);
  const double dyz = 0.5 * (S.yz + S.zy);
  const double dd  = dxx * dxx + dyy * dyy + dzz * dzz + 2.0 * (dxy * dxy + dxz * dxz + dyz * dyz);
  return sqrt(0.5 * dd);
}

double shearRate(const mat9r &L) {
  mat9r D;
  D.xx = L.xx;
  D.yy = L.yy;
  D.zz = L.zz;
  D.xy = D.yx = 0.5 * (L.xy + L.yx);
  D.xz = D.zx = 0.5 * (L.xz + L.zx);
  D.yz = D.zy = 0.5 * (L.yz + L.zy);
  const double s = sqrtJ2(D); // = sqrt(0.5 dev(D):dev(D))
  return 2.0 * s;
}

double shearRate2D(const mat4r &L) {
  mat9r L3;
  L3.xx = L.xx;
  L3.xy = L.xy;
  L3.yx = L.yx;
  L3.yy = L.yy;
  L3.zz = 0.0;
  return shearRate(L3);
}

double equivalentShearStrain(const mat4r &F) {
  // Left Cauchy-Green tensor B = F F^T, symmetric positive definite.
  const double b11 = F.xx * F.xx + F.xy * F.xy;
  const double b12 = F.xx * F.yx + F.xy * F.yy;
  const double b22 = F.yx * F.yx + F.yy * F.yy;

  // Its two eigenvalues. Only the invariants of the Hencky strain are needed,
  // so the eigenvectors never have to be formed.
  const double tr  = b11 + b22;
  const double det = b11 * b22 - b12 * b12;
  double disc      = tr * tr - 4.0 * det;
  if (disc < 0.0) { disc = 0.0; } // round-off on a nearly isotropic B
  const double root = sqrt(disc);

  double lp = 0.5 * (tr + root);
  double lm = 0.5 * (tr - root);
  if (lp <= 0.0 || lm <= 0.0) { return 0.0; } // degenerate F, nothing to say

  // Principal Hencky strains. The third one is zero: plane strain gives
  // F_zz = 1.
  const double e1 = 0.5 * log(lp);
  const double e2 = 0.5 * log(lm);
  const double m  = (e1 + e2) / 3.0;

  const double d1 = e1 - m;
  const double d2 = e2 - m;
  const double d3 = -m;

  return sqrt(2.0 * (d1 * d1 + d2 * d2 + d3 * d3));
}

double mobilisedSinPhi(const mat9r &S) {
  mat9r V;
  vec3r D;
  mat9r Sym = S;
  // The stress can carry a tiny antisymmetric part from the averaging; the
  // Jacobi rotation expects a symmetric matrix.
  Sym.xy = Sym.yx = 0.5 * (S.xy + S.yx);
  Sym.xz = Sym.zx = 0.5 * (S.xz + S.zx);
  Sym.yz = Sym.zy = 0.5 * (S.yz + S.zy);
  Sym.sym_eigen(V, D);

  double S1 = D.x;
  double S3 = D.x;
  if (D.y > S1) { S1 = D.y; }
  if (D.z > S1) { S1 = D.z; }
  if (D.y < S3) { S3 = D.y; }
  if (D.z < S3) { S3 = D.z; }

  const double sum = S1 + S3;
  if (sum <= 0.0) { return 0.0; }
  return (S1 - S3) / sum;
}

double sinPhiToTanPhi(double sinPhi) {
  if (sinPhi < 0.0 || sinPhi >= 1.0) { return 0.0; }
  return sinPhi / sqrt(1.0 - sinPhi * sinPhi);
}

mat9r planeStrainProjection(const mat9r &S) {
  mat9r P = S;
  P.xz = P.zx = 0.0;
  P.yz = P.zy = 0.0;
  return P;
}

mat9r fullStress(const mat4r &stress2D, double szz, double sxz, double syz) {
  mat9r S;
  S.xx = stress2D.xx;
  S.xy = stress2D.xy;
  S.yx = stress2D.yx;
  S.yy = stress2D.yy;
  S.zz = szz;
  S.xz = S.zx = sxz;
  S.yz = S.zy = syz;
  return S;
}

} // namespace ScalarTools
