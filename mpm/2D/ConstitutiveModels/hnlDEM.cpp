#include "hnlDEM.hpp"

#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>

#include <PBC3D.hpp>
#include <factory.hpp>

static Registrar<ConstitutiveModel, hnlDEM> registrar("hnlDEM");
std::string hnlDEM::getRegistrationName() { return std::string("hnlDEM"); }

hnlDEM::hnlDEM() {}

void hnlDEM::read(std::istream& is) {
  is >> fileName;
}
void hnlDEM::write(std::ostream& os) { os << fileName << '\n'; }

void hnlDEM::updateStrainAndStress(MPMbox& MPM, size_t p) {
  int* I = &(MPM.Elem[MPM.MP[p].e].I[0]);

  // Get the total strain increment from node velocities
  vec2r vn;
  mat4 dstrain;
  std::cout << "Entering hnlDEM::udateStrainAndStress" << std::endl;
  // MPM.MP[p].PBC.saveConf(p, "tata");
  for (int r = 0; r < element::nbNodes; r++) {
    if (MPM.nodes[I[r]].mass > MPM.tolmass) {
      vn = MPM.nodes[I[r]].q / MPM.nodes[I[r]].mass;
    } else
      continue;

    dstrain.xx += (vn.x * MPM.MP[p].gradN[r].x) * MPM.dt;
    dstrain.xy += 0.5 * (vn.x * MPM.MP[p].gradN[r].y + vn.y * MPM.MP[p].gradN[r].x) * MPM.dt;
    dstrain.yy += (vn.y * MPM.MP[p].gradN[r].y) * MPM.dt;
  }
  std::cout << "hnlDEM: dstrain computed" << std::endl;
  dstrain.yx = dstrain.xy;
  MPM.MP[p].strain += dstrain;
  MPM.MP[p].PBC.tmax += MPM.dt;
  h.reset(0);
  h.xx = MPM.MP[p].PBC.Cell.vh.xx;
  h.xy = MPM.MP[p].PBC.Cell.vh.xy;
  h.yx = MPM.MP[p].PBC.Cell.vh.yx;
  h.yy = MPM.MP[p].PBC.Cell.vh.yy;
  F.reset(0);
  F.xx = MPM.MP[p].F.xx;
  F.xy = MPM.MP[p].F.xy;
  F.yx = MPM.MP[p].F.yx;
  F.yy = MPM.MP[p].F.yy;
  F.zz = 1;
  std::cout << "hnlDEM: F h computed" << std::endl;
  MPM.MP[p].PBC.Load.TransformationGradient(h, F, MPM.dt);
  std::cout << "hnlDEM: DEM : Load.TransformationGradient ok " << std::endl;
  //char fn[256];
  // sprintf(fn, "%s_DEM", fileName);
  std::cout << "hnlDEM: DEM : calling integrate " << std::endl;
  // MPM.MP[p].PBC.saveConf(p, "toto");
  // MPM.MP[p].PBC.integrate(fn);
  std::cout << "hnlDEM: DEM : integrate ok " << std::endl;
  // Elastic stress
  MPM.MP[p].stress.xx += MPM.MP[p].PBC.Sig.xx;
  MPM.MP[p].stress.yy += MPM.MP[p].PBC.Sig.yy;
  MPM.MP[p].stress.xy += MPM.MP[p].PBC.Sig.xy;
  MPM.MP[p].stress.yx += MPM.MP[p].PBC.Sig.yx;
}

double hnlDEM::getYoung() { return 0; }
