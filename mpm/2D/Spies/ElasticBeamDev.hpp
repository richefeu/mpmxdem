#ifndef ELASTIC_BEAM_DEV_HPP
#define ELASTIC_BEAM_DEV_HPP

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

#endif /* end of include guard: ELASTIC_BEAM_DEV_HPP */
