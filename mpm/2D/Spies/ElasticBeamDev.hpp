#pragma once

#include "Spy.hpp"

#include <vector>


struct ElasticBeamDev : public Spy {
  void read(std::istream& is);

  void exec();
  void record();
  void end();

 private:
  std::string filename;
  std::ofstream file;

  double KinEnergyTot;
};
