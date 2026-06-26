#pragma once

#include "Spy.hpp"

#include <string>

#include "ElementSelector.hpp"
#include "mat4.hpp"

struct DPCPlot : public Spy {
  void read(std::istream& is);
  void exec();
  void record();
  void end();

 private:
  size_t MP_id;
  std::string filename;
  std::ofstream file;
  mat4r MPStress;
  mat4r MPStrain;
  double P;
  double Q;
  
};

