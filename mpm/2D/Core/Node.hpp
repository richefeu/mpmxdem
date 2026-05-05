#pragma once

// 
// Header file for the Node struct.
// 

#include <cstddef>

#include "mat4.hpp"
#include "vec2.hpp"

struct node {
  vec2r pos;                    // Position
  vec2r q;                      // Momentum (mass * velocity)
  vec2r qdot;                   // Derivative of Momentum
  vec2r f;                      // Force (volumic and internal)
  vec2r fb;                     // Interfacial force
  double mass{0.0};             // Mapped Mass
  bool xfixed{false};           // Null-velocity boolean along x
  bool yfixed{false};           // Null-velocity boolean along y
  mat4r stress;                 // Node-stress (used e.g. for smoothing)
  double outOfPlaneStress{0.0}; // third principal stress
  vec2r vel;                    // Node-velocity (used e.g. for smoothing)
  size_t number{0};             // An identifier number

  // operator used to compare. This allows me to use std::sort in a vector made of nodes
  bool operator<(const node &other) const {
    return number < other.number;
  }

  node(); // Ctor
};
