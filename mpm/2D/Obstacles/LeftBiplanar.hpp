#ifndef LEFTBIPLANAR_HPP_C1EB5D8D
#define LEFTBIPLANAR_HPP_C1EB5D8D

#include "Obstacle.hpp"

struct LeftBiplanar : public Obstacle {
  vec2r from1, to1, from2, to2, center, cArc;
  vec2r n1, t1, nArc, tArc, n2, t2;
  double l1, l2, R, angle, anglep0;
  int contactWith;

  virtual void read(std::istream& is);
  virtual void checkProximity(MPMbox& MPM);
  virtual int touch(MaterialPoint& MP, double& dn);
  virtual void getContactFrame(MaterialPoint& MP, vec2r& n, vec2r& t);
  virtual int addVtkPoints(std::vector<vec2r>& coords);
};

#endif /* end of include guard: LEFTBIPLANAR_HPP_C1EB5D8D */
