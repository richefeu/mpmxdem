#pragma once

#include "VtkOutput.hpp"

struct raw_totalStress : public VtkOutput {
  void save(std::ostream& os);
};
