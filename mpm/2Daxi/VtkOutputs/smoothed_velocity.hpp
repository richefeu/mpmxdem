#pragma once

#include "VtkOutput.hpp"

struct smoothed_velocity : public VtkOutput {
  void save(std::ostream& os);
};
