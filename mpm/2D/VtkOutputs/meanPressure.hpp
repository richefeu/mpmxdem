#ifndef MEANPRESSURE_HPP_5345A7C8
#define MEANPRESSURE_HPP_5345A7C8

#include "VtkOutput.hpp"

struct meanPressure : public VtkOutput {
  void save(std::ostream& os);
};

#endif /* end of include guard: MEANPRESSURE_HPP_5345A7C8 */
