#ifndef WORK_HPP
#define WORK_HPP

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

  slicedRange<double> Range;
  std::vector<double> Wn, Wt, Wint;

  double Wn_tot, Wt_tot, Wint_tot, Wp_tot;
};

#endif /* end of include guard: WORK_HPP */
