// spring-dashpot model

#include "frictionalViscoElastic.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

#include "factory.hpp"
static Registrar<BoundaryForceLaw, frictionalViscoElastic> registrar("frictionalViscoElastic");

void frictionalViscoElastic::computeForces(MPMbox& MPM, size_t o) {

  double kn, kt, en2, mu, viscosity;
  size_t g1, g2;
  for (size_t nn = 0; nn < MPM.Obstacles[o]->Neighbors.size(); ++nn) {
    size_t pn = MPM.Obstacles[o]->Neighbors[nn].PointNumber;
    double dn;
    int contact = MPM.Obstacles[o]->touch(MPM.MP[pn], dn);
    // if it returns -1 there is no contact, else it returns a positive number

    if (contact >= 0) {  // Check if there is contact
      g1 = (size_t)(MPM.MP[pn].groupNb);
      g2 = MPM.Obstacles[o]->group;
      kn = MPM.dataTable.get(MPM.id_kn, g1, g2);
      kt = MPM.dataTable.get(MPM.id_kt, g1, g2);
      // en2 = MPM.dataTable.get(MPM.id_en2, g1, g2);
      mu = MPM.dataTable.get(MPM.id_mu, g1, g2);
      viscosity = MPM.dataTable.get(MPM.id_viscosity, g1, g2);

      vec2r N, T;
      MPM.Obstacles[o]->getContactFrame(MPM.MP[pn], N, T);

      // === Normal force
      if (dn < 0.0) {  // overlapping
        // TODO: take into account the velocity of the obstacle (velobst - velmp or viceversa)
        double normalVel = MPM.MP[pn].vel * N;
        double visc = 2.0 * sqrt(MPM.MP[pn].mass * kn);
        MPM.Obstacles[o]->Neighbors[nn].fn = -kn * dn - visc * normalVel;
      }

      MPM.Obstacles[o]->Neighbors[nn].dn = dn;

      // === Friction force
      vec2r lever = MPM.MP[pn].pos - MPM.Obstacles[o]->pos;
      vec2r zCrossLever(-lever.y, lever.x);
      double delta_dt = ((MPM.MP[pn].pos - MPM.MP[pn].prev_pos) -
                         MPM.dt * (MPM.Obstacles[o]->vel + MPM.Obstacles[o]->vrot * zCrossLever)) *
                        T;

      MPM.Obstacles[o]->Neighbors[nn].ft += -kt * delta_dt;
      double threshold = mu * MPM.Obstacles[o]->Neighbors[nn].fn;

      if (MPM.Obstacles[o]->Neighbors[nn].ft > threshold) MPM.Obstacles[o]->Neighbors[nn].ft = threshold;
      if (MPM.Obstacles[o]->Neighbors[nn].ft < -threshold) MPM.Obstacles[o]->Neighbors[nn].ft = -threshold;

      // === Resultant force
      vec2r f = MPM.Obstacles[o]->Neighbors[nn].fn * N + MPM.Obstacles[o]->Neighbors[nn].ft * T;
      MPM.MP[pn].contactf = -f;  // useful for display
      MPM.MP[pn].f += f;
      MPM.Obstacles[o]->force -= f;

      // === Resultant moment (only on the obstacle)
      MPM.Obstacles[o]->mom += cross(lever, -f);
    } else {
      MPM.Obstacles[o]->Neighbors[nn].dn = 0.0;
      MPM.Obstacles[o]->Neighbors[nn].dt = 0.0;
      MPM.Obstacles[o]->Neighbors[nn].fn = 0.0;
      MPM.Obstacles[o]->Neighbors[nn].ft = 0.0;
      MPM.MP[pn].contactf.reset();
    }  // end if else touch
  }    // end for-loop over neighbors nn
}
