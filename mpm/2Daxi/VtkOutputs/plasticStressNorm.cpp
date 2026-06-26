#include "plasticStressNorm.hpp"

#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>

#include <factory.hpp>
static Registrar<VtkOutput, plasticStressNorm> registrar("plasticStressNorm");

void plasticStressNorm::save(std::ostream& os) {
  os << std::endl << "SCALARS plasticStressNorm float 1" << std::endl;
  os << "LOOKUP_TABLE default" << std::endl;
  // matrix norm (frobenius norm) https://en.wikipedia.org/wiki/Matrix_norm
  for (size_t i = 0; i < box->MP.size(); ++i) {
    mat4 temp = box->MP[i].plasticStress * box->MP[i].plasticStress.transpose();
    os << sqrt(temp.xx + temp.yy) << std::endl;
  }
}
