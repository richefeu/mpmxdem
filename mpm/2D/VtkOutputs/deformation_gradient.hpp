#ifndef DEFORMATION_GRADIENT_HPP_DA68A4E3
#define DEFORMATION_GRADIENT_HPP_DA68A4E3

#include "VtkOutput.hpp"

struct deformation_gradient : public VtkOutput {
  void save(std::ostream& os);
};

#endif /* end of include guard: DEFORMATION_GRADIENT_HPP_DA68A4E3 */
