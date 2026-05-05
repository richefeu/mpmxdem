#pragma once

#include "Obstacle.hpp"

struct Circle : public Obstacle {
  double R{0.0};

  std::string getRegistrationName();
  virtual void read(std::istream& is);
  virtual void write(std::ostream& os);
  virtual void checkProximity(MPMbox& MPM);
  virtual int touch(MaterialPoint& MP, double& dn);
  virtual void getContactFrame(MaterialPoint& MP, vec2r& N, vec2r& T);
  virtual bool inside(vec2r& x);
};
