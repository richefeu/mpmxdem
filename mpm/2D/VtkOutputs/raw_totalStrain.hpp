#ifndef RAW_TOTALSTRAIN_HPP_6A376F1E
#define RAW_TOTALSTRAIN_HPP_6A376F1E

#include "VtkOutput.hpp"

struct raw_totalStrain : public VtkOutput {
  void save(std::ostream& os);
};

#endif /* end of include guard: RAW_TOTALSTRAIN_HPP_6A376F1E */
