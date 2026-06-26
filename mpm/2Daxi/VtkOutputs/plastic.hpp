#pragma once

#include "VtkOutput.hpp"

struct plastic : public VtkOutput {
  void save(std::ostream& os);
};
