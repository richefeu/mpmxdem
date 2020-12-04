#ifndef SMOOTHED_VELOCITY_HPP_17930ECA
#define SMOOTHED_VELOCITY_HPP_17930ECA

#include "VtkOutput.hpp"

struct smoothed_velocity : public VtkOutput {
  void save(std::ostream& os);
};

#endif /* end of include guard: SMOOTHED_VELOCITY_HPP_17930ECA */
