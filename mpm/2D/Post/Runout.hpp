#pragma once

#include <fstream>
#include <string>

#include "PostProcessor.hpp"

//
// Runout of a column collapse, one line per configuration.
//
//     Post Runout <fileName> <xWall> <yFloor> <L0> <H0>
//
// xWall and yFloor locate the corner the column rests against, L0 and H0 are
// the initial width and height used to normalise. The front position and the
// deposit height are taken from the four corners of the deformed Material
// Points (ProcessedDataMP::corner), not from their centres, so that the size
// of a point is accounted for.
//
struct Runout : public PostProcessor {
  void read(std::istream &is);
  void begin();
  void exec();
  void end();

private:
  std::string filename;
  std::ofstream file;

  double xWall{0.0};
  double yFloor{0.0};
  double L0{1.0};
  double H0{1.0};
};
