#ifndef IRREGULARPOLYGON_HPP_C1EB5D8D
#define IRREGULARPOLYGON_HPP_C1EB5D8D

#include "Obstacle.hpp"
#include "Polygon.hpp"

struct IrregularPolygon : public Polygon {
  std::vector<vec2r> vectorsCentertoVertices;

  virtual void read(std::istream& is);
  virtual int touch(MaterialPoint& MP, double& dn);
  void checkProximity(MPMbox& MPM);
  virtual int addVtkPoints(std::vector<vec2r>& coords);
  // virtual void createPolygon(std::vector<vec2r> & vect);
  double calculateArea();
  vec2r calculateCentroid();
  void movePolygon(vec2r& centroid);
  double calculateInertia(double& density);
};

#endif /* end of include guard: IRREGULARPOLYGON2_HPP_C1EB5D8D */
