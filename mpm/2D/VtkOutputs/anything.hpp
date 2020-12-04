#ifndef ANYTHING_HPP_5345A7C8
#define ANYTHING_HPP_5345A7C8

#include "VtkOutput.hpp"

struct anything : public VtkOutput {
  void save(std::ostream& os);
};

#endif /* end of include guard: ANYTHING_HPP_5345A7C8 */
