#include "KelvinVoigt.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

std::string KelvinVoigt::getRegistrationName() { return std::string("KelvinVoigt"); }

KelvinVoigt::KelvinVoigt(double young, double poisson) : Young(young), Poisson(poisson), C(young, poisson), eta(0.0) {}

void KelvinVoigt::read(std::istream& is) {
  is >> Young >> Poisson >> eta;
  C.set(Young, Poisson);
}
void KelvinVoigt::write(std::ostream& os) { os << Young << ' ' << Poisson << ' ' << eta << '\n'; }

void KelvinVoigt::updateStrainAndStress(MPMbox& MPM, size_t p) {
  size_t* I = &(MPM.Elem[MPM.MP[p].e].I[0]);

  // Get the total strain increment from node velocities
  mat4r dstrain;
  for (size_t r = 0; r < element::nbNodes; r++) {
    dstrain.xx += (MPM.nodes[I[r]].vel.x * MPM.MP[p].gradN[r].x) * MPM.dt;
    dstrain.xy +=
        0.5 * (MPM.nodes[I[r]].vel.x * MPM.MP[p].gradN[r].y + MPM.nodes[I[r]].vel.y * MPM.MP[p].gradN[r].x) * MPM.dt;
    dstrain.yy += (MPM.nodes[I[r]].vel.y * MPM.MP[p].gradN[r].y) * MPM.dt;
  }
  dstrain.yx = dstrain.xy;

  MPM.MP[p].strain += dstrain;
  MPM.MP[p].deltaStrain = dstrain;

  // clang-format off
  mat9r Sigma(MPM.MP[p].stress.xx, MPM.MP[p].stress.xy, 0.0,
              MPM.MP[p].stress.yx, MPM.MP[p].stress.yy, 0.0,
              0.0,                 0.0,                 MPM.MP[p].outOfPlaneStress);

  mat9r dstrain3x3(dstrain.xx, dstrain.xy, 0.0,
                   dstrain.yx, dstrain.yy, 0.0,
                   0.0,        0.0,        0.0);
  // clang-format on

  // sigma_n = (sigma_{n-1} - eta*epsdot_{n-1}) + C:deps_n + eta*epsdot_n
  //
  // The viscous part is INSTANTANEOUS: it is eta times the current strain rate,
  // and has no business being accumulated. It used to be simply added to the
  // running total at every step, which summed to eta*epsilon_total/dt -- an
  // extra stiffness of modulus eta/dt, all the larger as the time step is
  // small, and not a damper at all. The contribution of the previous step is
  // therefore removed before the new one is added.
  Sigma.xx -= MPM.MP[p].viscousStress.xx;
  Sigma.xy -= MPM.MP[p].viscousStress.xy;
  Sigma.yx -= MPM.MP[p].viscousStress.yx;
  Sigma.yy -= MPM.MP[p].viscousStress.yy;
  Sigma.zz -= MPM.MP[p].outOfPlaneViscousStress;

  Sigma += C.getStress(dstrain3x3); // elastic part, incremental

  mat9r viscous = (eta / MPM.dt) * dstrain3x3; // viscous part, instantaneous
  Sigma += viscous;

  MPM.MP[p].viscousStress.xx        = viscous.xx;
  MPM.MP[p].viscousStress.xy        = viscous.xy;
  MPM.MP[p].viscousStress.yx        = viscous.yx;
  MPM.MP[p].viscousStress.yy        = viscous.yy;
  MPM.MP[p].outOfPlaneViscousStress = viscous.zz;

  MPM.MP[p].stress.xx = Sigma.xx;
  MPM.MP[p].stress.yy = Sigma.yy;
  MPM.MP[p].stress.xy = Sigma.xy;
  MPM.MP[p].stress.yx = Sigma.yx;
  MPM.MP[p].outOfPlaneStress = Sigma.zz;
}

double KelvinVoigt::getYoung() { return Young; }

double KelvinVoigt::getPoisson() { return Poisson; }
