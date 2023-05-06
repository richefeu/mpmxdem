#ifndef stressCorrectionNORM_HPP_5345A7C8
#define stressCorrectionNORM_HPP_5345A7C8

#include "VtkOutput.hpp"

struct plasticStressNorm : public VtkOutput {
  void save(std::ostream& os);
};

#endif /* end of include guard: PLASTIC_HPP_5345A7C8 */
