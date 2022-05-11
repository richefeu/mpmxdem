#ifndef NODE_HPP
#define NODE_HPP

#include "mat4.hpp"
#include "vec2.hpp"

struct node {
  vec2r pos;    // Position
  vec2r q;      // Momentum (mass * velocity)
  vec2r qdot;   // Derivative of Momentum
  vec2r f;      // Force (volumic and internal)
  vec2r fb;     // Interfacial force
  double mass;  // Mapped Mass
  bool xfixed;  // Null-velocity boolean along x
  bool yfixed;  // Null-velocity boolean along y
  mat4 stress;  // Node-stress (used e.g. for smoothing)
  double sigma3; // third principal stress
  vec2r vel;    // Node-velocity (used e.g. for smoothing)
  int number;   // An identifier number

  // operator used to compare. This allows me to use std::sort in a vector made of nodes
  bool operator<(const node& other) const { return number < other.number; }

  node();  // Ctor
};

#endif /* end of include guard: NODE_HPP */
