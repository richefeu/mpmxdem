#include "hnlDEML.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

#include "PBC3D.hpp"

#include "factory.hpp"
static Registrar<ConstitutiveModel, hnlDEML> registrar("hnlDEML");
std::string hnlDEML::getRegistrationName() { return std::string("hnlDEML"); }

hnlDEML::hnlDEML() {}

void hnlDEML::read(std::istream& is) { is >> fileName >> timeBondReactivation >> bondingFactor; }

void hnlDEML::write(std::ostream& os) {
  os << fileName << ' ' << timeBondReactivation << ' ' << bondingFactor << '\n';
}

// The elastic properties cannot be get that way, so, as a convention, -1 is returned
double hnlDEML::getYoung() { return -1.0; }
double hnlDEML::getPoisson() { return -1.0; }

void hnlDEML::init(MaterialPoint& MP) {
  MP.PBC = new PBC3Dbox;
  MP.PBC->loadConf(fileName.c_str());
  MP.isDoubleScale = true;
}

void hnlDEML::updateStrainAndStress(MPMbox& MPM, size_t p) {
  int* I = &(MPM.Elem[MPM.MP[p].e].I[0]);
  // Get the total strain increment from node velocities
  vec2r vn;
  mat4r dstrain;
  char fnamea[256];
  int col_i;
  int row_i;
  for (int r = 0; r < element::nbNodes; r++) {
    dstrain.xx += (MPM.nodes[I[r]].vel.x * MPM.MP[p].gradN[r].x) * MPM.dt;
    dstrain.xy +=
        0.5 * (MPM.nodes[I[r]].vel.x * MPM.MP[p].gradN[r].y + MPM.nodes[I[r]].vel.y * MPM.MP[p].gradN[r].x) * MPM.dt;
    dstrain.yy += (MPM.nodes[I[r]].vel.y * MPM.MP[p].gradN[r].y) * MPM.dt;
  }

  dstrain.yx = dstrain.xy;
  MPM.MP[p].strain += dstrain;
  MPM.MP[p].deltaStrain = dstrain;

  mat4r prev_F_inv = MPM.MP[p].prev_F;
  prev_F_inv.inverse();
  mat4r Finc2D = MPM.MP[p].F * prev_F_inv;

  // remember here that MPM is 2D and DEM is 3D
  mat9r Finc3D;
  Finc3D.xx = Finc2D.xx;
  Finc3D.xy = Finc2D.xy;
  Finc3D.yx = Finc2D.yx;
  Finc3D.yy = Finc2D.yy;
  Finc3D.zz = 1.0; // assuming plane strain
  
  mat9r SigAvg;
  MPM.MP[p].PBC->transform(Finc3D, MPM.dt, MPM.NHL.minDEMstep, MPM.NHL.rateAverage, SigAvg);
  
  col_i = p % MPM.Grid.Nx;
  row_i = floor(p / MPM.Grid.Nx);
  if (MPM.t >= timeBondReactivation - MPM.dt && MPM.t <= timeBondReactivation + MPM.dt) {
    MPM.MP[p].PBC->dn0/=bondingFactor;
    MPM.MP[p].PBC->dt0/=bondingFactor;
    MPM.MP[p].PBC->numericalDampingCoeff = 0.0;
  }
  if (MPM.step % MPM.confPeriod == 0 && col_i % MPM.DEMPeriod == 0 && row_i % MPM.DEMPeriod == 0) {
    sprintf(fnamea, "%s/DEM_MP%zu_t%i", MPM.result_folder.c_str(), p, MPM.iconf);
    MPM.MP[p].PBC->saveConf(fnamea);
    MPM.MP[p].PBC->iconf++;
  }
  
  // Stress
  // !!! (Sign convention is opposed) !!!
  MPM.MP[p].stress.xx = -SigAvg.xx;
  MPM.MP[p].stress.xy = -SigAvg.xy;
  MPM.MP[p].stress.yx = -SigAvg.yx;
  MPM.MP[p].stress.yy = -SigAvg.yy;
  MPM.MP[p].outOfPlaneStress = -SigAvg.zz;
}
