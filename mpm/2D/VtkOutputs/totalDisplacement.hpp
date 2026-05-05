#pragma once

#include "VtkOutput.hpp"

struct totalDisplacement : public VtkOutput {
  void save(std::ostream& os);
};
