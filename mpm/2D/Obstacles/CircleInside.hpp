#ifndef CIRCLEINSIDE_HPP_C1EB5D8D
#define CIRCLEINSIDE_HPP_C1EB5D8D

#include "Obstacle.hpp"

struct CircleInside : public Obstacle {
  double R;
  CircleInside();
  CircleInside(vec2r Pos, double Radius);
  virtual void read(std::istream& is);
  virtual void checkProximity(MPMbox& MPM);
  virtual int touch(MaterialPoint& MP, double& dn);
  virtual void getContactFrame(MaterialPoint& MP, vec2r& n, vec2r& t);
  virtual int addVtkPoints(std::vector<vec2r>& coords);
  virtual bool MPisInside(MaterialPoint& MP);
};

#endif /* end of include guard: CIRCLEINSIDE_HPP_C1EB5D8D */
