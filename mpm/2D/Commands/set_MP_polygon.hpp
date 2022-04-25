#ifndef SET_MP_POLYGON_HPP
#define SET_MP_POLYGON_HPP

#include <string>
#include <vector>

#include "vec2.hpp"

#include "Command.hpp"

struct set_MP_polygon : public Command {
  void read(std::istream&);
  void exec();

 private:
  bool onSegment(vec2r p, vec2r q, vec2r r);
  double orientation(vec2r p, vec2r q, vec2r r);
  bool doIntersect(vec2r p1, vec2r q1, vec2r p2, vec2r q2);
  bool isInside(std::vector<vec2r> polygon, int n, vec2r p);
  //double etaDamping;

  std::string modelName;
  int groupNb, nbVertices;
  vec2r vertex;
  double rho, size;
  std::vector<vec2r> vertices;
};

#endif /* end of include guard: SET_MP_POLYGON_HPP */
