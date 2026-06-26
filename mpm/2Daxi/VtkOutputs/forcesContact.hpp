#pragma once

#include "VtkOutput.hpp"

struct forcesContact : public VtkOutput {
  void save(std::ostream& os);
};
