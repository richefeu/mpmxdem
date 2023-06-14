#ifndef OBSTACLE_POLYGON_HPP
#define OBSTACLE_POLYGON_HPP

#include "Obstacle.hpp"

struct Polygon : public Obstacle {

  double R;
  int nVertices;
  std::vector<vec2r> verticePos;
  vec2r normal;
  vec2r tang;

  std::string getRegistrationName();
  virtual void read(std::istream& is);
  virtual void write(std::ostream& os);
  virtual void checkProximity(MPMbox& MPM);
  virtual int touch(MaterialPoint& MP, double& dn);
  virtual void getContactFrame(MaterialPoint& MP, vec2r& n, vec2r& t);
  virtual bool inside(vec2r& x);
  bool pointinPolygon(vec2r& point, MaterialPoint& MP, double& testdn);
  virtual void createPolygon(std::vector<vec2r>& vect);
  virtual double Area();
};

#endif /* end of include guard: OBSTACLE_POLYGON2_HPP */
