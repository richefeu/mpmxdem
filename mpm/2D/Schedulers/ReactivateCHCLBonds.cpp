#include "ReactivateCHCLBonds.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

void ReactivateCHCLBonds::read(std::istream& is) { is >> bondingDistance >> timeBondReactivation; }

void ReactivateCHCLBonds::write(std::ostream& os) {
  os << "ReactivateCHCLBonds " << bondingDistance << ' ' << timeBondReactivation << '\n';
}

void ReactivateCHCLBonds::check() {
  if (box->t >= timeBondReactivation - box->dt && box->t <= timeBondReactivation + box->dt) {
    for (size_t p = 0; p < box->MP.size(); p++) {
      box->MP[p].PBC->ActivateBonds(bondingDistance, bondedStateDam);
    }
  }
}