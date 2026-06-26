#pragma once

#include "VtkOutput.hpp"

struct deformation_gradient : public VtkOutput {
  void save(std::ostream& os);
};
