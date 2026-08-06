// USF -- Update Stress First.
//
// The stress is computed at the BEGINNING of the step, from the nodal
// velocities obtained by mapping the Material Point momenta, and the internal
// forces are then built on that fresh stress. ModifiedLagrangian remains the
// reference scheme, in particular for double-scale computations (see below),
// but this one is maintained and usable.

#include "UpdateStressFirst.hpp"

#include "ConstitutiveModels/ConstitutiveModel.hpp"
#include "Core/Element.hpp"
#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"
#include "Core/Node.hpp"
#include "Obstacles/Obstacle.hpp"
#include "ShapeFunctions/ShapeFunction.hpp"
#include "Spies/Spy.hpp"

#include "PBC3D.hpp"

std::string UpdateStressFirst::getRegistrationName() {
  return std::string("UpdateStressFirst");
}

int UpdateStressFirst::advanceOneStep(MPMbox &MPM) {
  START_TIMER("USF step");

  // Defining aliases =============================
  std::vector<node> &nodes           = MPM.nodes;
  std::vector<size_t> &liveNodeNum   = MPM.liveNodeNum;
  std::vector<element> &Elem         = MPM.Elem;
  std::vector<MaterialPoint> &MP     = MPM.MP;
  std::vector<Obstacle *> &Obstacles = MPM.Obstacles;
  double &dt                         = MPM.dt;
  double &tolmass                    = MPM.tolmass;
  vec2r &gravity                     = MPM.gravity;
  // End of aliases ================================

  if (MPM.step == 0) { Logger::info("Running UpdateStressFirst"); }
  size_t *I; // use as node index

  // ==== Discard previous grid

  for (size_t n = 0; n < liveNodeNum.size(); n++) {
    nodes[liveNodeNum[n]].mass = 0.0;
    nodes[liveNodeNum[n]].q.reset();
    nodes[liveNodeNum[n]].qdot.reset();
    nodes[liveNodeNum[n]].f.reset();
    nodes[liveNodeNum[n]].fb.reset();
    nodes[liveNodeNum[n]].vel.reset();
  }

  MPM.number_MP_before_any_split = MPM.MP.size();

  // shapeN / shapeGradN suivent le nombre de points
  MPM.resizeShapeArrays();

  // ==== Reset the resultant forces on MPs
  // (velGrad is cleared by MPMbox::updateVelocityGradient)
  for (size_t p = 0; p < MP.size(); p++) { MP[p].f.reset(); }

  // ==== Delete computed resultants (force and moment) of rigid obstacles
  for (size_t o = 0; o < Obstacles.size(); ++o) { OneStep::resetDEM(Obstacles[o], MPM.gravity); }

  // ==== Compute interpolation values
  for (size_t p = 0; p < MP.size(); p++) { MPM.shapeFunction->computeInterpolationValues(MPM, p); }

  // ==== Update Vector of node indices
  MPM.updateLiveNodeList();

  // ==== Move the rigid obstacles according to their mode of driving
  for (size_t o = 0; o < Obstacles.size(); ++o) { OneStep::moveDEM1(Obstacles[o], dt); }

  // ==== Initialize grid state (mass and momentum)
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);

    const double *Np = MPM.N(p);
    for (size_t r = 0; r < element::nbNodes; r++) {
      // Nodal mass
      nodes[I[r]].mass += Np[r] * MP[p].mass;
      // Nodal momentum
      nodes[I[r]].q += Np[r] * MP[p].mass * MP[p].vel;

      // Blocked DOFs (at nodes)
      if (nodes[I[r]].xfixed) { nodes[I[r]].q.x = 0.0; }
      if (nodes[I[r]].yfixed) { nodes[I[r]].q.y = 0.0; }
    }
  }

  // ==== Nodal velocities  (A)

  for (size_t n = 0; n < liveNodeNum.size(); n++) {
    if (nodes[liveNodeNum[n]].mass > tolmass)
      nodes[liveNodeNum[n]].vel = nodes[liveNodeNum[n]].q / nodes[liveNodeNum[n]].mass;
    else nodes[liveNodeNum[n]].vel.reset();
  }

  // ==== Deformation gradient and Volume (C)
  // This is also where the DEM time-step limiter acts, through
  // MPMbox::limitTimeStepForDEM: it comes before dt is used further down, so
  // double-scale computations are consistent with this scheme.
  MPM.updateTransformationGradient();
  for (size_t p = 0; p < MP.size(); p++) { MP[p].vol = MP[p].F.det() * MP[p].vol0; }
  OneStep::updateDensityFromVolume(MPM);

  // ==== Update strain and stress (CHCL models included)
  OneStep::updateStrainAndStress(MPM);

  // ==== Compute internal and external forces
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    const double *Np = MPM.N(p);
    const vec2r *gNp = MPM.gradN(p);
    for (size_t r = 0; r < element::nbNodes; r++) {
      // Internal forces
      nodes[I[r]].f += -MP[p].vol * (MP[p].stress * gNp[r]);
      // External forces (gravity)
      nodes[I[r]].f += MP[p].mass * gravity * Np[r];
    }
  }

  // ==== Boundary Conditions
  for (size_t o = 0; o < Obstacles.size(); ++o) { Obstacles[o]->boundaryForceLaw->computeForces(MPM, o); }

  // Updating free boundary conditions
  for (size_t o = 0; o < Obstacles.size(); ++o) { OneStep::moveDEM2(Obstacles[o], dt); }

  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    const double *Np = MPM.N(p);
    for (size_t r = 0; r < element::nbNodes; r++) { nodes[I[r]].fb += MP[p].f * Np[r]; }
  }

  // ==== Compute rate of momentum and update nodes
  for (size_t n = 0; n < liveNodeNum.size(); n++) {
    // sum of boundary and volume forces:
    nodes[liveNodeNum[n]].qdot = nodes[liveNodeNum[n]].fb + nodes[liveNodeNum[n]].f;
    if (nodes[liveNodeNum[n]].xfixed) nodes[liveNodeNum[n]].qdot.x = 0.0;
    if (nodes[liveNodeNum[n]].yfixed) nodes[liveNodeNum[n]].qdot.y = 0.0;

    nodes[liveNodeNum[n]].q += nodes[liveNodeNum[n]].qdot * dt; // newline! we were not updating the q (21-02-2017)
  }

  // ==== Update velocities of the MPs (FLIP/PIC blending)
  OneStep::updateMPVelocity(MPM);

  // ==== Update positions of the MPs
  for (size_t p = 0; p < MP.size(); p++) {
    I              = &(Elem[MP[p].e].I[0]);
    MP[p].prev_pos = MP[p].pos;
    double invmass;
    const double *Np = MPM.N(p);
    for (size_t r = 0; r < element::nbNodes; r++) {
      if (nodes[I[r]].mass > tolmass) {
        invmass = 1.0 / nodes[I[r]].mass;
        MP[p].pos += dt * Np[r] * nodes[I[r]].q * invmass;
      }
    }
  }

  // ==== Update the corner positions of the MPs
  if (MPM.needMPCorners) {
    for (size_t p = 0; p < MP.size(); p++) { MP[p].updateCornersFromF(); }
  }

  return 0;
}
