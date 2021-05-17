#ifndef LINE_HPP
#define LINE_HPP

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
  virtual int addVtkPoints(std::vector<vec2r>& coords);
};

#endif /* end of include guard: LINE_HPP */
