#pragma once

#include <iostream>
#include <vector>

#include "BoundaryForceLaw/BoundaryForceLaw.hpp"
#include "Core/Neighbor.hpp"
#include "vec2.hpp"
#include <vector>

class MPMbox;
struct MaterialPoint;
struct BoundaryForceLaw;
#define IS_FREE 0
#define FREEZE 1
#define IMPOSE_VELOCITY 2
#define IMPOSE_FORCE 3

struct Obstacle {
  int group{0};          // group for interaction parameters
  double securDist{0.0}; // security distance for detecting contact with MPs (seen as a disk)
  int drive_mode{FREEZE};
  
  double mass{0.0}; // mass is requiered when free motion is allowed
  double I{0.0};    // Inertia is requiered when rotation free motion is allowed

  vec2r pos;   // position
  vec2r vel;   // velocity
  vec2r acc;   // acceleration
  vec2r force; // resultant force
  vec2r normal;

  int steps{0};  // if > 1 -> vx and vy are each imposed during multiple steps (useful for loading then unloading)
  std::vector<vec2r> impVels;    // Vector to store imposed velocities
  std::vector<double> stepTimes; // Vector to store the duration of the steps as fractions of the total time
  std::vector<double> impForces;
  double damp;

  double rot{0.0};  // angular position
  double vrot{0.0}; // angular velocity
  double arot{0.0}; // angular acceleration
  double mom{0.0};  // resultant moment

  BoundaryForceLaw *boundaryForceLaw{nullptr};

  std::vector<Neighbor> Neighbors; // MP neighbors 
  virtual std::string getRegistrationName()                           = 0;
  virtual void read(std::istream &is)                                 = 0;
  virtual void write(std::ostream &os)                                = 0;
  virtual void checkProximity(MPMbox &MPM)                            = 0;
  virtual int touch(MaterialPoint &MP, double &dn)                    = 0;
  virtual void getContactFrame(MaterialPoint &MP, vec2r &n, vec2r &t) = 0;
  virtual void updateImposedVelocity(MPMbox &MPM)                     = 0;

  virtual bool inside(vec2r &x);

  Obstacle();
  virtual ~Obstacle();
};
