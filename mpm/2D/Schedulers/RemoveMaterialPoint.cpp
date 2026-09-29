#include "RemoveMaterialPoint.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"
#include "ConstitutiveModels/ConstitutiveModel.hpp"
#include "Obstacles/Obstacle.hpp"

void RemoveMaterialPoint::read(std::istream& is) { is >> CMkey >> removeTime; }

void RemoveMaterialPoint::write(std::ostream& os) {
  os << "RemoveMaterialPoint " << CMkey << ' ' << removeTime << '\n';
}

//
// Remove every Material Point driven by the constitutive model 'CMkey'.
//
// The points are held in a contiguous vector, so removing some of them shifts
// all the ones that follow: any index into that vector taken before the removal
// designates another point afterwards, or no point at all.
//
// The obstacles keep exactly such indices, in Neighbor::PointNumber. And the
// order inside a step is checkProximity(), then the schedulers, then
// advanceOneStep(): the lists are built before this function runs and used
// after it, in the very same step. The guard at the top of MPMbox::run --
// MP.size() != number_MP_before_any_split -- only reacts at the NEXT step, once
// the damage is done.
//
// The lists are therefore rebuilt here, right after the compaction. The contact
// history is lost for the surviving points, which is the correct outcome: the
// history is restored by matching point NUMBERS, and those numbers no longer
// mean anything in the old lists.
//
void RemoveMaterialPoint::check() {

  if (removeTime >= box->t && removeTime <= box->t + box->dt) {
    std::vector<MaterialPoint> MP_swap;
    MP_swap.reserve(box->MP.size());
    for (size_t i = 0; i < box->MP.size(); i++) {
      if (box->MP[i].constitutiveModel->key != CMkey) {
        MP_swap.push_back(box->MP[i]);
      } else if (box->MP[i].isDoubleScale == true && box->MP[i].PBC != nullptr) {
        // Each Material Point owns its DEM sample (double-scale points are never
        // split, see B11), so this is the last reference to it.
        delete box->MP[i].PBC;
        box->MP[i].PBC = nullptr;
      }
    }
    MP_swap.swap(box->MP);

    for (size_t o = 0; o < box->Obstacles.size(); o++) { box->Obstacles[o]->Neighbors.clear(); }
    box->checkProximity();
  }
}