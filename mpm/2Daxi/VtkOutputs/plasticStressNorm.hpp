#pragma once

#include "VtkOutput.hpp"

struct plasticStressNorm : public VtkOutput {
  void save(std::ostream& os);
};

