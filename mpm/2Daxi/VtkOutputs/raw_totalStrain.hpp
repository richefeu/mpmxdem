#pragma once

#include "VtkOutput.hpp"

struct raw_totalStrain : public VtkOutput {
  void save(std::ostream& os);
};

