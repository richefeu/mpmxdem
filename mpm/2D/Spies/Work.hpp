#pragma once

#include "Spy.hpp"

#include <vector>

#include "slicedRange.hpp"

struct Work : public Spy {
  void read(std::istream& is);

  void exec();
  void record();
  void end();

 private:
  std::string filenameSlices;
  std::string filename;
  std::ofstream fileSlices;
  std::ofstream file;
  std::string plotType;

  slicedRange<double> Range;
  std::vector<double> Wn, Wt, Wint;

  double Xmin, Xmax;
  unsigned int nbSlices;
  double Wn_tot, Wt_tot, Wint_tot, Wp_tot, KEr_tot;
  bool start;
};
