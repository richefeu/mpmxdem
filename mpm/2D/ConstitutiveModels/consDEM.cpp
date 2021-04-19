#include "consDEM.hpp"
  
#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>

#include <factory.hpp>
#include <PBC3D.hpp>


static Registrar<ConstitutiveModel, consDEM> registrar("consDEM");

consDEM::consDEM(const char * fileName):dname(fileName){}


void consDEM::updateStrainAndStress(MPMbox& MPM, size_t p) {
  int* I = &(MPM.Elem[MPM.MP[p].e].I[0]);

  // Get the total strain increment from node velocities
  vec2r vn;
  mat4 dstrain;

  for (int r = 0; r < element::nbNodes; r++) {
    if (MPM.nodes[I[r]].mass > MPM.tolmass) {
      vn = MPM.nodes[I[r]].q / MPM.nodes[I[r]].mass;
    } else
      continue;
    //~ std::cout<<"q: "<<MPM.nodes[I[r]].q <<std::endl;
    dstrain.xx += (vn.x * MPM.MP[p].gradN[r].x) * MPM.dt;
    dstrain.xy += 0.5 * (vn.x * MPM.MP[p].gradN[r].y + vn.y * MPM.MP[p].gradN[r].x) * MPM.dt;
    dstrain.yy += (vn.y * MPM.MP[p].gradN[r].y) * MPM.dt;
  }
  dstrain.yx = dstrain.xy;

  MPM.MP[p].strain += dstrain;
  MPM.PBC[p].tmax +=MPM.dt;
  h.reset(0);
  h.xx=MPM.MP[p].velGrad.xx; 
  h.xy=MPM.MP[p].velGrad.xy;
  h.yx=MPM.MP[p].velGrad.yx;
  h.yy=MPM.MP[p].velGrad.yy; 
  F.reset(0);
  F.xx=MPM.MP[p].Fincrement.xx; 
  F.xy=MPM.MP[p].Fincrement.xy;
  F.yx=MPM.MP[p].Fincrement.yx;
  F.yy=MPM.MP[p].Fincrement.yy;
  F.zz=1;
  MPM.PBC[p].Load.TransformationGradient(h, F, 1);
  // Elastic stress
  MPM.MP[p].stress.xx += MPM.PBC[p].Sig.xx;
  MPM.MP[p].stress.yy += MPM.PBC[p].Sig.yy;
  MPM.MP[p].stress.xy += MPM.PBC[p].Sig.xy;
  MPM.MP[p].stress.yx += MPM.PBC[p].Sig.yx;
}

double consDEM::getYoung() {return 0;}
void consDEM::read(std::istream& is) {}

