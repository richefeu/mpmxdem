#pragma once

#include <string>

#include "PostProcessor.hpp"

//
// Per-Material-Point fields, one file per configuration.
//
//     Post MPFields <baseName> <xmin> <ymin> <xmax> <ymax>
//
// Writes '<baseName><confNum>.txt' holding one line per Material Point whose
// centre falls in the box. Give a box larger than the grid to take them all.
//
// This is the command-file version of the old 'See/cut.cpp'.
//
struct MPFields : public PostProcessor {
  void read(std::istream &is);
  void exec();

private:
  std::string baseName;

  double xmin{0.0};
  double ymin{0.0};
  double xmax{0.0};
  double ymax{0.0};
};
