#ifndef OBSTACLE_HPP
#define OBSTACLE_HPP

#include <iostream>
#include <vector>

#include "BoundaryForceLaw/BoundaryForceLaw.hpp"
#include "Core/Neighbor.hpp"
#include "vec2.hpp"

class MPMbox;
struct MaterialPoint;
struct BoundaryForceLaw;

struct Obstacle {
  int group{0};          // group for interaction parameters
  double securDist{0.0}; // security distance for detecting contact with MPs (seen as a disk)
  bool isFree{false};    // true -> x, y, and rot free; false -> vx and vy imposed, and vrot = 0

  double mass{0.0}; // mass is requiered when free motion is allowed
  double I{0.0};    // Inertia is requiered when rotation free motion is allowed

  vec2r pos;   // position
  vec2r vel;   // velocity
  vec2r acc;   // acceleration
  vec2r force; // resultant force

  double rot{0.0};  // angular position
  double vrot{0.0}; // angular velocity
  double arot{0.0}; // angular acceleration
  double mom{0.0};  // resultant moment

  BoundaryForceLaw *boundaryForceLaw;

  std::vector<Neighbor> Neighbors; // MP neighbors

  virtual std::string getRegistrationName()                           = 0;
  virtual void read(std::istream &is)                                 = 0;
  virtual void write(std::ostream &os)                                = 0;
  virtual void checkProximity(MPMbox &MPM)                            = 0;
  virtual int touch(MaterialPoint &MP, double &dn)                    = 0;
  virtual void getContactFrame(MaterialPoint &MP, vec2r &n, vec2r &t) = 0;

  virtual bool inside(vec2r &x);

  Obstacle();
  virtual ~Obstacle();
};

#endif /* end of include guard: OBSTACLE_HPP */
