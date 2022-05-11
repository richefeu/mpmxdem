#include "hnlDEM.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

#include "PBC3D.hpp"

#include "factory.hpp"
static Registrar<ConstitutiveModel, hnlDEM> registrar("hnlDEM");
std::string hnlDEM::getRegistrationName() { return std::string("hnlDEM"); }

hnlDEM::hnlDEM() {}

void hnlDEM::read(std::istream& is) {
  is >> fileName >> etaDamping >> timeBonds >> distBonds;
}
void hnlDEM::write(std::ostream& os) { os << fileName << ' ' << etaDamping << ' ' <<  timeBonds << ' ' << distBonds <<'\n'; }

double hnlDEM::getYoung() {return -1;}

void hnlDEM::init(MaterialPoint & MP) {
  MP.PBC = new PBC3Dbox;
  MP.PBC->loadConf(fileName.c_str());
  MP.ismicro = true;
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
    dstrain.xx += (MPM.nodes[I[r]].vel.x * MPM.MP[p].gradN[r].x) * MPM.dt;
    dstrain.xy += 0.5 * (MPM.nodes[I[r]].vel.x * MPM.MP[p].gradN[r].y + MPM.nodes[I[r]].vel.y * MPM.MP[p].gradN[r].x) * MPM.dt;
    dstrain.yy += (MPM.nodes[I[r]].vel.y * MPM.MP[p].gradN[r].y) * MPM.dt;
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
  MPM.MP[p].PBC->transform(Finc3D, MPM.dt,MPM.demstable,MPM.stablelength);
  col_i=p%MPM.Grid.Nx;
  row_i=floor(p/MPM.Grid.Nx);
  if( MPM.t >=timeBonds-MPM.dt && MPM.t <=timeBonds+MPM.dt){
  MPM.MP[p].PBC->mpmBonds(distBonds);
  }
  if (MPM.step % MPM.confPeriod == 0  && col_i%MPM.DEMPeriod ==0 && row_i%MPM.DEMPeriod ==0) {
    sprintf(fnamea, "%s/DEM_MP%zu_t%i", MPM.result_folder.c_str(), p,MPM.iconf);
    MPM.MP[p].PBC->saveConf(fnamea);
    MPM.MP[p].PBC->iconf++;
  }
  // Elastic stress
  // (Sign convention is opposed)
  MPM.MP[p].stress.xx = -MPM.MP[p].PBC->Sig.xx+etaDamping*MPM.MP[p].velGrad.xx; 
  MPM.MP[p].stress.xy = -MPM.MP[p].PBC->Sig.xy+0.5*etaDamping*(MPM.MP[p].velGrad.xy+MPM.MP[p].velGrad.yx);
  MPM.MP[p].stress.yx = -MPM.MP[p].PBC->Sig.yx+0.5*etaDamping*(MPM.MP[p].velGrad.xy+MPM.MP[p].velGrad.yx);
  MPM.MP[p].stress.yy = -MPM.MP[p].PBC->Sig.yy+etaDamping*MPM.MP[p].velGrad.yy;
  MPM.MP[p].sigma3=-MPM.MP[p].PBC->Sig.zz;
}

