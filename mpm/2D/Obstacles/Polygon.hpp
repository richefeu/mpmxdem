#ifndef POLYGON_HPP_C1EB5D8D
#define POLYGON_HPP_C1EB5D8D

#include "Obstacle.hpp"

struct Polygon : public Obstacle {
  
  double R;
  int nVertices;
  std::vector<vec2r> verticePos;
  vec2r normal;
  vec2r tang;

  virtual void read(std::istream& is);
  virtual void checkProximity(MPMbox& MPM);
  virtual int touch(MaterialPoint& MP, double& dn);
  virtual void getContactFrame(MaterialPoint& MP, vec2r& n, vec2r& t);
  virtual int addVtkPoints(std::vector<vec2r>& coords);
  virtual bool MPisInside(MaterialPoint& MP);
  bool pointinPolygon(int& nVertices, vec2r& point, std::vector<vec2r>& verticePos, MaterialPoint& MP, vec2r& tang,
                      vec2r& normal, double& testdn);
  bool edgeinMP(vec2r& edgePos, int& closestCorner, MaterialPoint& MP, vec2r& tang, vec2r& normal, double& testdn);
  virtual void createPolygon(std::vector<vec2r>& vect);
  virtual double Area();
};

#endif /* end of include guard: POLYGON2_HPP_C1EB5D8D */
