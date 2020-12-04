#ifndef PLASTIC_HPP_5345A7C8
#define PLASTIC_HPP_5345A7C8

#include "VtkOutput.hpp"

struct plastic : public VtkOutput {
  void save(std::ostream& os);
};

#endif /* end of include guard: PLASTIC_HPP_5345A7C8 */
