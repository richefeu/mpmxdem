#include "hnlDEM.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

#include "PBC3D.hpp"

#include "factory.hpp"
static Registrar<ConstitutiveModel, hnlDEM> registrar("hnlDEM");
std::string hnlDEM::getRegistrationName() { return std::string("hnlDEM"); }

hnlDEM::hnlDEM() {}

void hnlDEM::read(std::istream& is) {
  is >> fileName;
}
void hnlDEM::write(std::ostream& os) { os << fileName << '\n'; }

void hnlDEM::init(MaterialPoint & MP) {
  MP.PBC = new PBC3Dbox;
  MP.PBC->loadConf(fileName.c_str());
  MP.ismicro=true;
}

void hnlDEM::updateStrainAndStress(MPMbox& MPM, size_t p) {
  int* I = &(MPM.Elem[MPM.MP[p].e].I[0]);

  // Get the total strain increment from node velocities
  vec2r vn;
  mat4 dstrain;
  char fnamea[256];
  int col_i;
  int row_i;
  for (int r = 0; r < element::nbNodes; r++) {
    if (MPM.nodes[I[r]].mass > MPM.tolmass) {
      vn = MPM.nodes[I[r]].q / MPM.nodes[I[r]].mass;
    } else
      continue;

    dstrain.xx += (vn.x * MPM.MP[p].gradN[r].x) * MPM.dt;
    dstrain.xy += 0.5 * (vn.x * MPM.MP[p].gradN[r].y + vn.y * MPM.MP[p].gradN[r].x) * MPM.dt;
    dstrain.yy += (vn.y * MPM.MP[p].gradN[r].y) * MPM.dt;
  }
  
  dstrain.yx = dstrain.xy;
  MPM.MP[p].strain += dstrain;
  MPM.MP[p].deltaStrain = dstrain;
  
  mat4 prev_F_inv = MPM.MP[p].prev_F;
  prev_F_inv.inverse();
  mat4 Finc2D = MPM.MP[p].F*prev_F_inv;
  
  // remember here that MPM is 2D and DEM is 3D
  mat9r Finc3D;
  Finc3D.xx = Finc2D.xx;
  Finc3D.xy = Finc2D.xy;
  Finc3D.yx = Finc2D.yx;
  Finc3D.yy = Finc2D.yy;
  Finc3D.zz = 1.0; // assuming plane strain
  MPM.MP[p].PBC->transform(Finc3D, MPM.dt);
  col_i=p%MPM.Grid.Nx;
  row_i=floor(p/MPM.Grid.Nx);
  if (MPM.step % MPM.confPeriod == 0  && col_i%MPM.DEMPeriod ==0 && row_i%MPM.DEMPeriod ==0) {
    sprintf(fnamea, "%s/DEM_MP%zu_t%i", MPM.result_folder.c_str(), p,MPM.iconf);
    MPM.MP[p].PBC->saveConf(fnamea);
    MPM.MP[p].PBC->iconf++;
  }
  // Elastic stress
  // (Sign convention is different)
  MPM.MP[p].stress.xx = -MPM.MP[p].PBC->Sig.xx; 
  MPM.MP[p].stress.xy = -MPM.MP[p].PBC->Sig.xy;
  MPM.MP[p].stress.yx = -MPM.MP[p].PBC->Sig.yx;
  MPM.MP[p].stress.yy = -MPM.MP[p].PBC->Sig.yy;
}

double hnlDEM::getYoung() { return 0; }
