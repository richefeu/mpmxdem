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
      // PBC is null for every single-scale point: only a numerically
      // homogeneised law owns a DEM sample with bonds to reactivate.
      if (box->MP[p].isDoubleScale == false || box->MP[p].PBC == nullptr) { continue; }
      box->MP[p].PBC->ActivateBonds(bondingDistance, bondedStateDam);
    }
  }
}