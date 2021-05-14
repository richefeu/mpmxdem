#ifndef ADD_MP_SHALLOWPATH_HPP
#define ADD_MP_SHALLOWPATH_HPP

#include <string>
#include <vector>

#include "Command.hpp"
#include <vec2.hpp>

struct add_MP_ShallowPath : public Command {
  void read(std::istream& is);
  void exec();

 private:
  double lineEquation(const vec2r& point1, const vec2r& point2, const double xpos);

  std::string modelName;
  int groupNb;
  int nbPathPoints;
  vec2r pathPoint;
  double height;
  double rho;
  double size;
  std::vector<vec2r> pathPoints;
};

#endif /* end of include guard: ADD_MP_SHALLOWPATH_HPP */
