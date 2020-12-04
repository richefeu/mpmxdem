#ifndef RAW_VELOCITY_HPP_A95AFB09
#define RAW_VELOCITY_HPP_A95AFB09

#include "VtkOutput.hpp"

struct raw_velocity : public VtkOutput {
  void save(std::ostream& os);
};

#endif /* end of include guard: RAW_VELOCITY_HPP_A95AFB09 */
