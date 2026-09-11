#pragma once

#include "Spy.hpp"

#include <string>
#include <vector>

#include "ElementSelector.hpp"
#include "mat4.hpp"

struct DPCPlot : public Spy {
  void read(std::istream& is);
  void exec();
  void record();
  void end();
  std::vector<size_t> tracked_MPs;

 private:
  int nMP;
  bool start = true;

  mat4r MPStress;
  mat4r MPStrain;

  std::vector<std::string> PQ_filenames;
  std::vector<std::ofstream*> PQ_files;
  std::vector<std::string> parameters_filenames;
  std::vector<std::ofstream*> parameters_files;
  
  std::vector<double> Pvals;
  std::vector<double> Qvals;
  std::vector<double> Pbvals;
  std::vector<double> betavals;
  std::vector<double> Rvals;
  std::vector<double> dvals;
  std::vector<double> Evals;
  std::vector<double> Nuvals;
  std::vector<double> RDvals;
};

