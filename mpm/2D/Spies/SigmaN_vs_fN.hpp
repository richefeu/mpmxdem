#pragma once

#include "Spy.hpp"

#include <string>

#include "ElementSelector.hpp"
#include "mat4.hpp"

struct SigmaN_vs_fN : public Spy {
  void read(std::istream& is);
  void exec();
  void record();
  void end();

 private:
  size_t MP_id;
  std::string filename;
  std::ofstream file;
  vec2r f ;
  double l;
  double sigma_n;

};

