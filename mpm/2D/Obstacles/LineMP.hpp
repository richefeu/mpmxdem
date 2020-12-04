#ifndef LINEMP_HPP_C1EB5D8D
#define LINEMP_HPP_C1EB5D8D

#include "Obstacle.hpp"

struct LineMP : public Obstacle {

  double len;
  vec2r n, t;

  virtual void read(std::istream& is);
  virtual void checkProximity(MPMbox& MPM);
  virtual int touch(MaterialPoint& MP, double& dn);
  virtual void getContactFrame(MaterialPoint& MP, vec2r& n, vec2r& t);
  virtual int addVtkPoints(std::vector<vec2r>& coords);
};

#endif /* end of include guard: LINEMP_HPP_C1EB5D8D */
