#include "totalDisplacement.hpp"

#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>

#include <factory.hpp>
static Registrar<VtkOutput, totalDisplacement> registrar("totalDisplacement");

void totalDisplacement::save(std::ostream& os) {
  os << std::endl << "VECTORS totalDisplacement float " << std::endl;
  for (size_t i = 0; i < box->MP.size(); ++i) {
    os << box->MP[i].pos - box->MP[i].prev_pos << " 0" << std::endl;
  }
}