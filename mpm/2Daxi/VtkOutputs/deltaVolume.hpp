#pragma once

#include "VtkOutput.hpp"

struct deltaVolume : public VtkOutput {
  void save(std::ostream& os);
};
