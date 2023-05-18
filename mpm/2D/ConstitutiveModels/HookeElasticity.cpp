#include "HookeElasticity.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

std::string HookeElasticity::getRegistrationName() { return std::string("HookeElasticity"); }

HookeElasticity::HookeElasticity(double young, double poisson) : Young(young), Poisson(poisson), C(young, poisson) {}

void HookeElasticity::read(std::istream& is) {
  is >> Young >> Poisson;
  C.set(Young, Poisson);
}
void HookeElasticity::write(std::ostream& os) { os << Young << ' ' << Poisson << '\n'; }

// This simple Hooke linear elasticity around a given state, and for a given velocity gradient et time increment
void HookeElasticity::updateStrainAndStress(MPMbox& MPM, size_t p) {
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

  Sigma += C.getStress(dstrain3x3);
  // clang-format on

  MPM.MP[p].stress.xx = Sigma.xx;
  MPM.MP[p].stress.yy = Sigma.yy;
  MPM.MP[p].stress.xy = Sigma.xy;
  MPM.MP[p].stress.yx = Sigma.yx;
  MPM.MP[p].outOfPlaneStress = Sigma.zz;
}

double HookeElasticity::getYoung() { return Young; }

double HookeElasticity::getPoisson() { return Poisson; }
