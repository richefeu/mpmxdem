#ifndef OBSTACLE_CIRCLE_HPP
#define OBSTACLE_CIRCLE_HPP

#include "Obstacle.hpp"

struct Circle : public Obstacle {
  double R;

  std::string getRegistrationName();
  virtual void read(std::istream& is);
  virtual void write(std::ostream& os);
  virtual void checkProximity(MPMbox& MPM);
  virtual int touch(MaterialPoint& MP, double& dn);
  virtual void getContactFrame(MaterialPoint& MP, vec2r& n, vec2r& t);
  virtual bool MPisInside(MaterialPoint& MP);
};

#endif /* end of include guard: OBSTACLE_CIRCLE_HPP */
