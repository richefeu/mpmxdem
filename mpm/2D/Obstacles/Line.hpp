#ifndef OBSTACLE_LINE_HPP
#define OBSTACLE_LINE_HPP

#include "Obstacle.hpp"

struct Line : public Obstacle {
  double len;
  vec2r n, t;
  
  std::string getRegistrationName();
  virtual void read(std::istream& is);
  virtual void write(std::ostream& os);
  virtual void checkProximity(MPMbox& MPM);
  virtual int touch(MaterialPoint& MP, double& dn);
  virtual void getContactFrame(MaterialPoint& MP, vec2r& n, vec2r& t);
};

#endif /* end of include guard: OBSTACLE_LINE_HPP */
