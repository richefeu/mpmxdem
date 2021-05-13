#ifndef CIRCLE_HPP
#define CIRCLE_HPP

#include "Obstacle.hpp"

struct Circle : public Obstacle {
  double R;

  virtual void read(std::istream& is);
  virtual void checkProximity(MPMbox& MPM);
  virtual int touch(MaterialPoint& MP, double& dn);
  virtual void getContactFrame(MaterialPoint& MP, vec2r& n, vec2r& t);
  virtual int addVtkPoints(std::vector<vec2r>& coords);
  virtual bool MPisInside(MaterialPoint& MP);
};

#endif /* end of include guard: CIRCLE_HPP */
