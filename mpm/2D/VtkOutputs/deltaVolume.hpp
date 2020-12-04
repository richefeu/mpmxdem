#ifndef DELTAVOLUME_HPP_5EA269CC
#define DELTAVOLUME_HPP_5EA269CC

#include "VtkOutput.hpp"

struct deltaVolume : public VtkOutput {
  void save(std::ostream& os);
};

#endif /* end of include guard: DELTAVOLUME_HPP_5EA269CC */
