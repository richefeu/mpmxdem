#ifndef TOTALDISPLACEMENT_HPP_1C1A3535
#define TOTALDISPLACEMENT_HPP_1C1A3535

#include "VtkOutput.hpp"

struct totalDisplacement : public VtkOutput {
  void save(std::ostream& os);
};

#endif /* end of include guard: TOTALDISPLACEMENT_HPP_1C1A3535 */
