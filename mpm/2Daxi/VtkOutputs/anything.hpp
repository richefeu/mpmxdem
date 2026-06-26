#pragma once

#include "VtkOutput.hpp"

struct anything : public VtkOutput {
  void save(std::ostream& os);
};
