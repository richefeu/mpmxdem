#ifndef INTERACTION_HPP
#define INTERACTION_HPP

#include <cstdlib>  // for size_t

#include "vec3.hpp"

const int noContactState = 0;
const int contactState = 1;
const int bondedState = 2;
const int bondedStateDam = 3;

/// Data of Interaction
struct Interaction {
  size_t i;     ///< ID-number of the first particle
  size_t j;     ///< ID-number of the second particle
  double gap0;  ///< Initial gap of the bonded link
  vec3r n;      ///< Normal unit vector of contact (components are expressed in the world frame)
  double fn;    ///< Normal force (scalar value)
  double fn_elas; ///< The elastic part of normal force
  double fn_bond; ///< The bond part of the normal force
  vec3r ft;  ///< Tangential force (components are expressed in the world frame)
  vec3r ft_fric; ///< Frictionnal tangent force (at contact)
  vec3r ft_bond; ///< The bond part of tangent force 
  vec3r dt_fric;
  vec3r dt_bond;
  vec3r drot_bond;
  vec3r drot_fric;
  vec3r mom;  ///< Torque vector (due to bond angular resistance)
  vec3r mom_fric;
  vec3r mom_bond;

  double dampn;  ///< Precomputed vicuous damping coefficient in normal direction
  double dampt;  ///< Precomputed vicuous damping coefficient in tangent direction
  int state;     ///< State of the contact (can be noContactState, contactState or bondedState)
  double D;      ///< Damage parameter

  Interaction();
  Interaction(size_t I, size_t J, double Dampn, double Dampt);

  bool operator<(const Interaction& other) const;
};

#endif /* end of include guard: INTERACTION_HPP */
