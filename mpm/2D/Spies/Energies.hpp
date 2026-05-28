#pragma once

#include "Spy.hpp"

#include <vector>

struct Energies : public Spy {
  void read(std::istream& is);

  void exec();
  void record();
  void end();

 private:
  std::string filename;
  std::ofstream file;

  double Ec, Ep, Eel, Epl;
};
