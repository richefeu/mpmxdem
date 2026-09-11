#pragma once

#include "Spy.hpp"

#include <string>

#include "ElementSelector.hpp"
#include "mat4.hpp"

struct DPCPlotMean : public Spy {
  void read(std::istream& is);
  void exec();
  void record();
  void end();

 private:
  bool start = true;

  mat4r MPStress;
  mat4r MPStrain;

  std::string PQ_filename;
  std::ofstream PQ_file;
  std::string parameters_filename;
  std::ofstream parameters_file;
  
  double meanP;
  double meanQ;
  double meanPb;
  double meanbeta;
  double meanR;
  double meand;
  double meanE;
  double meanNu;
  double meanRD;
  
};

