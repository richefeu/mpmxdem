#ifndef SMOOTHED_TOTALSTRESS_HPP_DA68A4E3
#define SMOOTHED_TOTALSTRESS_HPP_DA68A4E3

#include "VtkOutput.hpp"

struct smoothed_totalStress : public VtkOutput {
  void save(std::ostream& os);
};

#endif /* end of include guard: SMOOTHED_TOTALSTRESS_HPP_DA68A4E3 */
