#include "RemoveMaterialPoint.hpp"

#include <Core/MPMbox.hpp>
#include "Core/MaterialPoint.hpp"
#include "ConstitutiveModels/ConstitutiveModel.hpp"

void RemoveMaterialPoint::read(std::istream& is) { is >> CMkey >> removeTime; }

void RemoveMaterialPoint::write(std::ostream& os) {
  os << "RemoveMaterialPoint " << CMkey << ' ' << removeTime << '\n';
}

void RemoveMaterialPoint::check() {

  if (removeTime >= box->t && removeTime <= box->t + box->dt) {
    std::vector<MaterialPoint> MP_swap;
	  for (size_t i = 0; i < box->MP.size(); i++) {
      if (box->MP[i].constitutiveModel->key != CMkey) {
        MP_swap.push_back(box->MP[i]);
      }
    }
    MP_swap.swap(box->MP);
  }
	
}