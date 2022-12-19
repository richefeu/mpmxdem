#include "hnlDEMcv.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

#include "PBC3D.hpp"

//#include "factory.hpp"
//static Registrar<ConstitutiveModel, hnlDEMcv> registrar("hnlDEMcv");
std::string hnlDEMcv::getRegistrationName() { return std::string("hnlDEMcv"); }

hnlDEMcv::hnlDEMcv() {}

void hnlDEMcv::read(std::istream& is) { is >> fileName >> timeBondchange >> bondingfactor; }

void hnlDEMcv::write(std::ostream& os) {
  os << fileName << ' ' << timeBondchange << ' ' << bondingfactor << '\n';
}

// The elastic properties cannot be get that way, so, as a convention, -1 is returned
double hnlDEMcv::getYoung() { return -1.0; }
double hnlDEMcv::getPoisson() { return -1.0; }

void hnlDEMcv::init(MaterialPoint& MP) {
  MP.PBC = new PBC3Dbox;
  MP.PBC->loadConf(fileName.c_str());
  MP.isDoubleScale = true;
}

void hnlDEMcv::updateStrainAndStress(MPMbox& MPM, size_t p) {
  size_t* I = &(MPM.Elem[MPM.MP[p].e].I[0]);
  // Get the total strain increment from node velocities
  vec2r vn;
  mat4r dstrain;
  char fnamea[256];
  size_t col_i;
  size_t row_i;
  for (size_t r = 0; r < element::nbNodes; r++) {
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
  
  col_i = static_cast<size_t>(p % MPM.Grid.Nx);
  row_i = static_cast<size_t>(floor((double)p / (double)MPM.Grid.Nx));
  if (MPM.t >= timeBondchange - MPM.dt && MPM.t <= timeBondchange + MPM.dt) {
    //MPM.MP[p].PBC->fn0/=bondingfactor;
    //MPM.MP[p].PBC->ft0/=bondingfactor;
    //MPM.MP[p].PBC->mom0/=bondingfactor;
    MPM.MP[p].PBC->dn0/=bondingfactor;
    MPM.MP[p].PBC->dt0/=bondingfactor;
    MPM.MP[p].PBC->drot0/=bondingfactor;
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
