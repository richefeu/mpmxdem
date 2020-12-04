#ifndef RAW_TOTALSTRESS_HPP_03DD6060
#define RAW_TOTALSTRESS_HPP_03DD6060

#include "VtkOutput.hpp"

struct raw_totalStress : public VtkOutput {
  void save(std::ostream& os);
};

#endif /* end of include guard: RAW_TOTALSTRESS_HPP_03DD6060 */
