#pragma once

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
  int groupNb{0};
  int nbPathPoints{0};
  vec2r pathPoint;
  double height{0.0};
  double rho{0.0};
  double size{0.0};
  std::vector<vec2r> pathPoints;
};

