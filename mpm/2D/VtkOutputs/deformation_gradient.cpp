#include "deformation_gradient.hpp"

#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>

#include <factory.hpp>
static Registrar<VtkOutput, deformation_gradient> registrar("deformation_gradient");

void deformation_gradient::save(std::ostream& os) {
  os << std::endl << "TENSORS deformation_gradient float" << std::endl;
  for (size_t p = 0; p < box->MP.size(); ++p) {
    os << box->MP[p].F.xx << " " << box->MP[p].F.xy << " 0" << std::endl;
    os << box->MP[p].F.yx << " " << box->MP[p].F.yy << " 0" << std::endl;
    os << "0 0 0" << std::endl;
  }
}
