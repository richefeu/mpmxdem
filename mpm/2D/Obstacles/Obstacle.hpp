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
  int group;
  double securDist;
  bool isFree;  // true -> x, y, and rot free; false -> vx and vy imposed, and vrot = 0

  double mass;
  double I;

  vec2r pos;
  vec2r vel;
  vec2r acc;
  vec2r force;

  double rot;
  double vrot;
  double arot;
  double mom;
  BoundaryForceLaw* boundaryForceLaw;

  std::vector<Neighbor> Neighbors; // MP neighbors

  virtual std::string getRegistrationName() = 0;
  virtual void read(std::istream& is) = 0;
  virtual void write(std::ostream& os) = 0;
  virtual void checkProximity(MPMbox& MPM) = 0;
  virtual int touch(MaterialPoint& MP, double& dn) = 0;
  virtual void getContactFrame(MaterialPoint& MP, vec2r& n, vec2r& t) = 0;

  virtual bool MPisInside(MaterialPoint& MP);

  Obstacle();
  virtual ~Obstacle();
};

#endif /* end of include guard: OBSTACLE_HPP */
