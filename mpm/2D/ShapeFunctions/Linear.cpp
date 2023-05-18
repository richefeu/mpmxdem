#include "Linear.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

std::string Linear::getRegistrationName() { return std::string("Linear"); }

Linear::Linear() {}

void Linear::computeInterpolationValues(MPMbox& MPM, size_t p) {
  MPM.MP[p].e = (size_t)(trunc(MPM.MP[p].pos.x / MPM.Grid.lx) + trunc(MPM.MP[p].pos.y / MPM.Grid.ly) * (double)MPM.Grid.Nx);
  size_t* I = &(MPM.Elem[MPM.MP[p].e].I[0]);

  double inv = 1.0 / (0.5 * MPM.Grid.lx);

  double inv2 = inv * inv;
  double dx0 = MPM.MP[p].pos.x - MPM.nodes[I[0]].pos.x - 0.5 * MPM.Grid.lx;
  double dy0 = MPM.MP[p].pos.y - MPM.nodes[I[0]].pos.y - 0.5 * MPM.Grid.lx;

  MPM.MP[p].N[0] = (1.0 - dx0 * inv) * (1.0 - dy0 * inv);
  MPM.MP[p].N[1] = (1.0 + dx0 * inv) * (1.0 - dy0 * inv);
  MPM.MP[p].N[2] = (1.0 - dx0 * inv) * (1.0 + dy0 * inv);
  MPM.MP[p].N[3] = (1.0 + dx0 * inv) * (1.0 + dy0 * inv);

  MPM.MP[p].gradN[0].x = (-inv + dy0 * inv2);
  MPM.MP[p].gradN[1].x = (inv - dy0 * inv2);
  MPM.MP[p].gradN[2].x = (-inv - dy0 * inv2);
  MPM.MP[p].gradN[3].x = (inv + dy0 * inv2);

  MPM.MP[p].gradN[0].y = (-inv + dx0 * inv2);
  MPM.MP[p].gradN[1].y = (-inv - dx0 * inv2);
  MPM.MP[p].gradN[2].y = (inv - dx0 * inv2);
  MPM.MP[p].gradN[3].y = (inv + dx0 * inv2);
}
