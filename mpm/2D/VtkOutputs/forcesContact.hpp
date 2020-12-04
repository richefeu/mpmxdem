#ifndef FORCESCONTACT_HPP_419582D7
#define FORCESCONTACT_HPP_419582D7

#include "VtkOutput.hpp"

struct forcesContact : public VtkOutput {
  void save(std::ostream& os);
};

#endif /* end of include guard: FORCESCONTACT_HPP_419582D7 */
