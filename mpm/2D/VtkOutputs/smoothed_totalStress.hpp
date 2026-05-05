#pragma once

#include "VtkOutput.hpp"

struct smoothed_totalStress : public VtkOutput {
  void save(std::ostream& os);
};
