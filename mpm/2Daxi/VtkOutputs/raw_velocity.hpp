#pragma once

#include "VtkOutput.hpp"

struct raw_velocity : public VtkOutput {
  void save(std::ostream& os);
};
