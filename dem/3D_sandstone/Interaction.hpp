#ifndef INTERACTION_HPP
#define INTERACTION_HPP

#include <cstdlib>  // for size_t

#include "vec3.hpp"

const int noContactState = 0;
const int contactState = 1;
const int bondedState = 2;
const int bondedStateDam = 3;

// Struct to hold information about an interaction between two particles
struct Interaction {
  size_t i{0};         ///< Unique identifier for the first particle involved in the interaction
  size_t j{0};         ///< Unique identifier for the second particle involved in the interaction
  double gap0{0.0};      ///< The initial gap between the two particles when a bond is first established
  vec3r n;          ///< The normal unit vector of the contact point, expressed in the world frame
  double fn{0.0};        ///< The total normal force acting on the particles
  double fn_elas{0.0};   ///< The elastic component of the normal force
  double fn_bond{0.0};   ///< The bond component of the normal force
  vec3r ft;         ///< The total tangential force acting on the particles, expressed in the world frame
  vec3r ft_fric;    ///< The frictional component of the tangential force at the contact point
  vec3r ft_bond;    ///< The bond component of the tangential force
  vec3r dt_fric;    ///< The increment of tangential relative displacement for friction
  vec3r dt_bond;    ///< The increment of tangential relative displacement for bonded link
  vec3r drot_bond;  ///< The increment of relative rotation angle for bonded link
  vec3r drot_fric;  ///< The increment of relative rotation angle for friction
  vec3r mom;        ///< The total moment vector acting on the particles due to bond angular resistance
  vec3r mom_fric;   ///< The frictional component of the torque vector
  vec3r mom_bond;   ///< The bond component of the torque vector

  double dampn{0.0};  ///< Precomputed viscous damping coefficient in the normal direction
  double dampt{0.0};  ///< Precomputed viscous damping coefficient in the tangent direction
  int state{noContactState};     ///< The current state of the contact (can be noContactState, contactState or bondedState)
  double D{0.0};      ///< The damage parameter, indicating the level of damage for a bonded link

  // Default constructor for Interaction
  Interaction();

  // Constructor for Interaction with initial particle IDs and damping coefficients
  Interaction(size_t I, size_t J, double Dampn, double Dampt);

  // Overloaded less-than operator for comparing Interactions
  bool operator<(const Interaction& other) const;
};

#endif /* end of include guard: INTERACTION_HPP */
