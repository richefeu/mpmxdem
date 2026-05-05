#pragma once

#include "VtkOutput.hpp"

struct meanPressure : public VtkOutput {
  void save(std::ostream& os);
};
