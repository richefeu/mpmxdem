#include "smoothed_velocity.hpp"

#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>

#include <factory.hpp>
static Registrar<VtkOutput, smoothed_velocity> registrar("smoothed_velocity");

void smoothed_velocity::save(std::ostream& os) {
  int* I;

  for (size_t p = 0; p < box->MP.size(); p++) {
    I = &(box->Elem[box->MP[p].e].I[0]);
    for (int r = 0; r < element::nbNodes; r++) {
      box->nodes[I[r]].vel += box->MP[p].N[r] * box->MP[p].mass * box->MP[p].vel / box->nodes[I[r]].mass;
    }
  }

  std::vector<vec2r> smoothVel(box->MP.size());
  for (size_t p = 0; p < box->MP.size(); p++) {
    I = &(box->Elem[box->MP[p].e].I[0]);
    for (int r = 0; r < element::nbNodes; r++) {
      smoothVel[p] += box->nodes[I[r]].vel * box->MP[p].N[r];
    }
  }

  os << std::endl << "VECTORS smoothed_velocity float" << std::endl;
  for (size_t i = 0; i < box->MP.size(); ++i) {
    os << smoothVel[i] << " 0" << std::endl;
  }
}