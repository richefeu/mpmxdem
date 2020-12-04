#include "smoothed_totalStress.hpp"

#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>

#include <factory.hpp>
static Registrar<VtkOutput, smoothed_totalStress> registrar("smoothed_totalStress");

void smoothed_totalStress::save(std::ostream& os) {
  int* I;
  for (size_t p = 0; p < box->MP.size(); p++) {
    I = &(box->Elem[box->MP[p].e].I[0]);
    for (int r = 0; r < element::nbNodes; r++) {
      box->nodes[I[r]].stress += box->MP[p].N[r] * box->MP[p].mass * box->MP[p].stress / box->nodes[I[r]].mass;
    }
  }

  std::vector<mat4> smoothStress(box->MP.size());
  for (size_t p = 0; p < box->MP.size(); p++) {
    I = &(box->Elem[box->MP[p].e].I[0]);
    for (int r = 0; r < element::nbNodes; r++) {
      smoothStress[p] += box->nodes[I[r]].stress * box->MP[p].N[r];
    }
  }

  os << std::endl << "TENSORS smoothed_totalStress float" << std::endl;
  for (size_t i = 0; i < box->MP.size(); ++i) {
    os << smoothStress[i].xx << " " << smoothStress[i].xy << " 0" << std::endl;
    os << smoothStress[i].yx << " " << smoothStress[i].yy << " 0" << std::endl;
    os << "0 0 0" << std::endl;
  }
}