#pragma once

#include "Obstacle.hpp"

struct Line : public Obstacle {
  double len;
  vec2r n; // unit normal oriented 'towards the left' when going from start to end 
	vec2r udir; // rename udir
  
  std::string getRegistrationName();
  virtual void read(std::istream& is);
  virtual void write(std::ostream& os);
  virtual void checkProximity(MPMbox& MPM);
  virtual int touch(MaterialPoint& MP, double& dn);
  virtual void getContactFrame(MaterialPoint& MP, vec2r& N, vec2r& T);
	virtual bool inside(vec2r& x);
};
