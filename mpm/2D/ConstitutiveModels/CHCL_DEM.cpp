#include "CHCL_DEM.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

#include "PBC3D.hpp"

std::string CHCL_DEM::getRegistrationName() { return std::string("CHCL_DEM"); }

CHCL_DEM::CHCL_DEM() {}

void CHCL_DEM::read(std::istream& is) { is >> fileName; }

void CHCL_DEM::write(std::ostream& os) { os << fileName << '\n'; }

// The elastic properties cannot be get that way, so, as a convention, -1 is returned
double CHCL_DEM::getYoung() { return -1.0; }
double CHCL_DEM::getPoisson() { return -1.0; }

void CHCL_DEM::init(MaterialPoint& MP) {
  MP.PBC = new PBC3Dbox;
  MP.PBC->loadConf(fileName.c_str());
  MP.isDoubleScale = true;

  // transfert the current stress
  // !!! (Sign convention is opposed between PBC3D and MPMbox) !!!
  MP.stress.xx = -MP.PBC->Sig.xx;
  MP.stress.xy = -MP.PBC->Sig.xy;
  MP.stress.yx = -MP.PBC->Sig.yx;
  MP.stress.yy = -MP.PBC->Sig.yy;
  MP.outOfPlaneStress = -MP.PBC->Sig.zz;
}

void CHCL_DEM::updateStrainAndStress(MPMbox& MPM, size_t p) {
  size_t* I = &(MPM.Elem[MPM.MP[p].e].I[0]);

  // Get the total strain increment from node velocities
  vec2r vn;
  mat4r dstrain;
  for (size_t r = 0; r < element::nbNodes; r++) {
    dstrain.xx += (MPM.nodes[I[r]].vel.x * MPM.MP[p].gradN[r].x) * MPM.dt;
    dstrain.xy +=
        0.5 * (MPM.nodes[I[r]].vel.x * MPM.MP[p].gradN[r].y + MPM.nodes[I[r]].vel.y * MPM.MP[p].gradN[r].x) * MPM.dt;
    dstrain.yy += (MPM.nodes[I[r]].vel.y * MPM.MP[p].gradN[r].y) * MPM.dt;
  }
  dstrain.yx = dstrain.xy;  // symmetic tensor

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
  Finc3D.zz = 1.0;  // assuming plane strain

  mat9r SigAvg;

  // clang-format off
  MPM.MP[p].PBC->transform(Finc3D, MPM.dt, 
	                         MPM.CHCL.minDEMstep, MPM.CHCL.rateAverage, MPM.CHCL.criticalDEMTimeStepFactor, 
													 SigAvg);
  // clang-format on

  // FIXME: maybe we should force the symmetry of the stress tensor
  // because it's ok for the DEM side, but probably not for MPM
  SigAvg.symmetrize();

  // Stress
  // !!! (Sign convention is opposed between PBC3D and MPMbox) !!!
  MPM.MP[p].stress.xx = -SigAvg.xx;
  MPM.MP[p].stress.xy = -SigAvg.xy;
  MPM.MP[p].stress.yx = -SigAvg.yx;
  MPM.MP[p].stress.yy = -SigAvg.yy;
  MPM.MP[p].outOfPlaneStress = -SigAvg.zz;
}
