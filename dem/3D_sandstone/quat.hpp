#ifndef QUAT_HPP
#define QUAT_HPP

/// @file
/// @brief Class for quaternions (specialized for body rotations in DEM)
/// @author Vincent Richefeu <Vincent.Richefeu@3sr-grenoble.fr>, Lab 3SR, Grenoble University
/// @date 2009-2010

#include "mat9.hpp"

/// Quaternions (specialized class for rotations)
class quat {
public:
  vec3r v;  ///< a vector
  double s; ///< a scalar

  static const quat identity;

  quat();
  quat(double X, double Y, double Z, double S);
  quat(const vec3r &V, const double S);

  quat &operator=(const quat &Q);

  quat &operator+=(const quat &a);
  quat &operator-=(const quat &a);
  quat &operator*=(double k);
  quat &operator/=(double k);

  friend quat operator*(const quat &q1, const quat &q2);

  vec3r operator*(const vec3r &V) const;

  quat dot(const vec3r &omega);
  void conjugate();
  quat get_conjugated() const;

  void reset();
  void set_axis_angle(const vec3r &V, double angle);
  void set(double X, double Y, double Z, double S);

  double get_angle() const;
  double get_Pitch() const;
  double get_Yaw() const;
  double get_Roll() const;
  vec3r get_axis() const;

  void set_from_to(const vec3r &V1, const vec3r &V2);
  void TwistSwingDecomp(const vec3r &V1, quat &twist, quat &swing);
  void SwingTwistDecomp(const vec3r &V1, quat &swing, quat &twist);

  double normalize();
  void get_rot_matrix(double M[]) const;
  void get_rot_matrix(mat9r &M) const;
  int set_rot_matrix(double m[]);
  mat9<double> rotate_diag_mat(const vec3r &u) const;
  vec3r rotate(const vec3r &u) const;
  vec3r unrotate(const vec3r &u) const;

  // --- input/output ---
  friend std::ostream &operator<<(std::ostream &pStr, const quat &Q);

  friend std::istream &operator>>(std::istream &pStr, quat &Q);
};

#endif /* end of include guard: QUAT_HPP */
