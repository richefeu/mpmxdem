#include "PBC3D.hpp"

int main(/*int argc, char const* argv[]*/) {
  PBC3Dbox box;

  box.Cell.h.reset();
  box.Cell.h.set_diag(1.0, 1.0, 1.0);  // reduced and world coordinate are the same

  // place the particles
  Particle P1;
  P1.radius = 0.1;
  P1.pos.set(0.4, 0.5, 0.5);
  box.Particles.push_back(P1);
  Particle P2;
  P2.radius = 0.1;
  P2.pos.set(0.6, 0.5, 0.5);
  box.Particles.push_back(P2);

  // set a damageable bond between the two particles
  box.Interactions.push_back(Interaction(0, 1, 0.0, 0.0));
  box.Interactions[0].gap0 = 0.0;
  box.Interactions[0].n.set(1.0, 0.0, 0.0);
  box.Interactions[0].state = bondedStateDam;

  // set the law parameters
  box.kn = 1.0e5;     ///< Normal stiffness (for compression/tension, bonded or not)
  box.kt = 1.0e5;     ///< Tangential stiffness (bonded or not)
  box.kr = 10.0;        ///< Angular stiffness (only for bonded links)
  box.mu = 0.5;       ///< Coefficent of friction
  box.mur = 0.4;      ///< Coefficient of "angular-friction"
  box.fcoh = 0.0;     ///< Cohesion force (strictly negative)
  box.zetaMax = 1.5;  ///< Can be seen as "dn_rupture / dn_dammage_starts"
  box.Kratio = 1;     ///< Ratio of particle stiffness over bond stiffness
  double w_bond = 1.0 / (box.Kratio + 1.0);

  // Solid cohesion
  box.fn0 = 20;     ///< Maximum normal force
  box.ft0 = 20;     ///< Maximum tangential force
  box.mom0 = 0.4;   ///< Maximum Torque
  box.powSurf = 5;  ///< Power used in the breakage surface

  box.dn0 = box.fn0 / (w_bond * box.kn);     ///< Maximum normal displacement
  box.dt0 = box.ft0 / (w_bond * box.kt);     ///< Maximum tangential displacement
  box.drot0 = box.mom0 / (w_bond * box.kr);  ///< Maximum angular rotation

  // set the loading
  double theta_tn = M_PI*0.5;  // rad
  vec3r axisRot(0.0, 0.0, 1.0);
  axisRot.normalize();
  double deplMax = P1.radius * 0.01;
  double rotMax = 0.25;
  int nsteps = 1000;

  double vtrans = deplMax / (double)nsteps;  // vtrans * nsteps * (dt=1) = deplMax
  double vrot = rotMax / (double)nsteps;     // vrot * nsteps * (dt=1) = rotMax
  box.Particles[1].vel.set(cos(theta_tn), sin(theta_tn), 0.0);
  box.Particles[1].vel.normalize();
  box.Particles[1].vel *= vtrans;
  box.Particles[1].vrot = vrot * axisRot;

  // A kind of integration loop
  box.computeSampleData();
  box.dt = 1.0;
  vec3r posIni = box.Particles[1].pos;
  for (int step = 0; step < nsteps; step++) {
    box.Particles[1].pos += box.dt * box.Particles[1].vel;

    // box.Particles[1].Q += ((box.Particles[1].Q.dot(box.Particles[1].vrot)) *= box.dt);
    // box.Particles[1].Q.normalize();
    box.Particles[1].Q.set_axis_angle(axisRot, 0.5 * step * vrot);
    box.Particles[0].Q.set_axis_angle(axisRot, -0.5 * step * vrot);

    box.computeForcesAndMoments();
    std::cout << box.Particles[1].pos - posIni << ' ' << step * vrot << ' ' << ' ' << box.Interactions[0].fn_bond << ' '
              << norm(box.Interactions[0].ft_bond) << ' ' << norm(box.Interactions[0].mom_bond) << '\n';
  }

  return 0;
}