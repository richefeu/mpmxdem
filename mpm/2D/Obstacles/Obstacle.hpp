#ifndef OBSTACLE_HPP_DEEAEE92
#define OBSTACLE_HPP_DEEAEE92

#include <BoundaryType/BoundaryType.hpp>
#include <Core/Neighbor.hpp>
#include <iostream>
#include <vec2.hpp>
#include <vector>

struct MPMbox;
struct MaterialPoint;
struct BoundaryType;

struct Obstacle {
  int group;
  double securDist;
  bool isFree;  // true = x, y, and rot free; false = vx and vy imposed, and vrot = 0

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
  BoundaryType* boundaryType;

  std::vector<Neighbor> Neighbors;

  virtual void read(std::istream& is) = 0;
  virtual void checkProximity(MPMbox& MPM) = 0;
  virtual int touch(MaterialPoint& MP, double& dn) = 0;
  virtual void getContactFrame(MaterialPoint& MP, vec2r& n, vec2r& t) = 0;
  virtual int addVtkPoints(std::vector<vec2r>& coords) = 0;

  virtual bool MPisInside(MaterialPoint& MP);

  Obstacle();
  virtual ~Obstacle();
};

#endif /* end of include guard: OBSTACLE_HPP_DEEAEE92 */
