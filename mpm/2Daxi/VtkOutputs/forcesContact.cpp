#include "forcesContact.hpp"

#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>

#include <factory.hpp>
static Registrar<VtkOutput, forcesContact> registrar("forcesContact");

void forcesContact::save(std::ostream& os) {
  os << std::endl << "VECTORS forcesContact float " << std::endl;
  for (size_t i = 0; i < box->MP.size(); ++i) {
    os << box->MP[i].contactf << " 0" << std::endl;
  }
}
