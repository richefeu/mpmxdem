#include "meanPressure.hpp"

#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>

#include <factory.hpp>
static Registrar<VtkOutput, meanPressure> registrar("meanPressure");

void meanPressure::save(std::ostream& os) {
  os << std::endl << "SCALARS meanPressure float 1" << std::endl;
  os << "LOOKUP_TABLE default" << std::endl;
  for (size_t i = 0; i < box->MP.size(); ++i) {
    os << 0.5 * (box->MP[i].stress.xx + box->MP[i].stress.yy) << std::endl;
  }
}