// spring-dashpot model

#include "frictionalViscoElastic.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

void frictionalViscoElastic::computeForces(MPMbox& MPM, size_t o) {
  double kn, kt, mu, viscRate;

  size_t g1, g2;
  for (size_t nn = 0; nn < MPM.Obstacles[o]->Neighbors.size(); ++nn) {
    size_t pn = MPM.Obstacles[o]->Neighbors[nn].PointNumber;
    double dn;
    MPM.Obstacles[o]->touch(MPM.MP[pn], dn);

    if (dn <= 0.0) {
      g1 = (size_t)(MPM.MP[pn].groupNb);
      g2 = MPM.Obstacles[o]->group;
      kn = MPM.dataTable.get(MPM.id_kn, g1, g2);
      kt = MPM.dataTable.get(MPM.id_kt, g1, g2);
      mu = MPM.dataTable.get(MPM.id_mu, g1, g2);
      viscRate = MPM.dataTable.get(MPM.id_viscRate, g1, g2);

      vec2r N, T;
      MPM.Obstacles[o]->getContactFrame(MPM.MP[pn], N, T);

      // === Normal force
      vec2r velRelative = MPM.MP[pn].vel - MPM.Obstacles[o]->vel;
      // remark: not really correct because the Obstable rotation is not accounted for.
      // This could be corrected by approximating that the MP position IS the contact position (see lever below)
      double normalVel = velRelative * N;
      double visc = viscRate * 2.0 * sqrt(MPM.MP[pn].mass * kn);
      MPM.Obstacles[o]->Neighbors[nn].fn = -kn * dn - visc * normalVel;
			
      MPM.Obstacles[o]->Neighbors[nn].dn = dn;

      // === Friction force			
      double delta_dt = (velRelative * T) * MPM.dt;
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
      vec2r lever = MPM.MP[pn].pos - MPM.Obstacles[o]->pos;
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
