// USL -- Update Stress Last.
//
// The internal forces are built on the stress of the previous step, and the
// stress is computed at the END of the step, from the nodal velocities obtained
// AFTER the momenta have been updated. Unlike ModifiedLagrangian, those end-of-
// step velocities come straight from the nodal momenta and are not mapped back
// from the Material Points -- that mapping is precisely what the 'Modified' of
// MUSL stands for.
//
// Reserve for double-scale computations: the deformation gradient, hence the
// DEM time-step limiter, is only known at the end of the step, so the limit
// applies to the NEXT one. Prefer ModifiedLagrangian or UpdateStressFirst
// there; a warning is issued at the first step.

#include "UpdateStressLast.hpp"

#include "ConstitutiveModels/ConstitutiveModel.hpp"
#include "Core/Element.hpp"
#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"
#include "Core/Node.hpp"
#include "Obstacles/Obstacle.hpp"
#include "ShapeFunctions/ShapeFunction.hpp"
#include "Spies/Spy.hpp"

#include "PBC3D.hpp"

std::string UpdateStressLast::getRegistrationName() { return std::string("UpdateStressLast"); }

int UpdateStressLast::advanceOneStep(MPMbox& MPM) {
  START_TIMER("USL step");

  // Defining aliases ==============================
  std::vector<node>& nodes = MPM.nodes;
  std::vector<size_t>& liveNodeNum = MPM.liveNodeNum;
  std::vector<element>& Elem = MPM.Elem;
  std::vector<MaterialPoint>& MP = MPM.MP;
  std::vector<Obstacle*>& Obstacles = MPM.Obstacles;
  double& dt = MPM.dt;
  double& tolmass = MPM.tolmass;
  vec2r& gravity = MPM.gravity;
  // End of aliases =================================

  if (MPM.step == 0) {
    Logger::info("Running UpdateStressLast");
    if (MPM.CHCL.hasDoubleScale == true) {
      Logger::warn("UpdateStressLast computes the deformation gradient at the end of the step, so the DEM "
                   "time-step limiter only constrains the NEXT step");
      Logger::warn("  Prefer ModifiedLagrangian, or UpdateStressFirst, for double-scale computations");
    }
  }
  size_t* I;  // use as node index

  // ==== Discard previous grid
  for (size_t n = 0; n < liveNodeNum.size(); n++) {
    nodes[liveNodeNum[n]].mass = 0.0;
    nodes[liveNodeNum[n]].q.reset();
    nodes[liveNodeNum[n]].qdot.reset();
    nodes[liveNodeNum[n]].f.reset();
    nodes[liveNodeNum[n]].fb.reset();
    nodes[liveNodeNum[n]].vel.reset();
  }

  MPM.number_MP_before_any_split = MP.size();
  
  // ==== Reset the resultant forces on MPs
  // (velGrad is cleared by MPMbox::updateVelocityGradient)
  for (size_t p = 0; p < MP.size(); p++) {
    MP[p].f.reset();
  }

  // ==== Delete computed resultants (force and moment) of rigid obstacles
  for (size_t o = 0; o < Obstacles.size(); ++o) {
    OneStep::resetDEM(Obstacles[o], MPM.gravity);
  }

  // ==== Compute interpolation values
  for (size_t p = 0; p < MP.size(); p++) {
    MPM.shapeFunction->computeInterpolationValues(MPM, p);
  }

  // ==== Update Vector of node indices
  std::set<size_t> sortedLive;
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    for (size_t r = 0; r < element::nbNodes; r++) {
      sortedLive.insert(I[r]);
    }
  }
  liveNodeNum.clear();
  std::copy(sortedLive.begin(), sortedLive.end(), std::back_inserter(liveNodeNum));

  // ==== Move the rigid obstacles according to their mode of driving
  for (size_t o = 0; o < Obstacles.size(); ++o) {
    OneStep::moveDEM1(Obstacles[o], dt);
  }

  // 1) ==== Initialize grid state (mass and momentum)
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);

    for (size_t r = 0; r < element::nbNodes; r++) {
      // Nodal mass
      nodes[I[r]].mass += MP[p].N[r] * MP[p].mass;
      // Nodal momentum
      nodes[I[r]].q += MP[p].N[r] * MP[p].mass * MP[p].vel;

      // Blocked DOFs (at nodes)
      if (nodes[I[r]].xfixed) {
        nodes[I[r]].q.x = 0.0;
      }
      if (nodes[I[r]].yfixed) {
        nodes[I[r]].q.y = 0.0;
      }
    }
  }

  // 1a) ==== Nodal velocities  (A)
  for (size_t n = 0; n < liveNodeNum.size(); n++) {
    if (nodes[liveNodeNum[n]].mass > tolmass)
      nodes[liveNodeNum[n]].vel = nodes[liveNodeNum[n]].q / nodes[liveNodeNum[n]].mass;
    else
      nodes[liveNodeNum[n]].vel.reset();
  }

  // 2) ==== Compute internal and external forces
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);

    for (size_t r = 0; r < element::nbNodes; r++) {
      // Internal forces
      nodes[I[r]].f += -MP[p].vol * (MP[p].stress * MP[p].gradN[r]);
      // External forces (gravity)
      nodes[I[r]].f += MP[p].mass * gravity * MP[p].N[r];
    }
  }

  for (size_t o = 0; o < Obstacles.size(); ++o) {
    Obstacles[o]->boundaryForceLaw->computeForces(MPM, o);
  }

  // Updating free boundary conditions
  for (size_t o = 0; o < Obstacles.size(); ++o) {
    OneStep::moveDEM2(Obstacles[o], dt);
  }

  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    for (size_t r = 0; r < element::nbNodes; r++) {
      nodes[I[r]].fb += MP[p].f * MP[p].N[r];
    }
  }

  // 3) ==== Compute rate of momentum and update nodes
  for (size_t n = 0; n < liveNodeNum.size(); n++) {
    // sum of boundary and volume forces:
    nodes[liveNodeNum[n]].qdot = nodes[liveNodeNum[n]].fb + nodes[liveNodeNum[n]].f;
    if (nodes[liveNodeNum[n]].xfixed) nodes[liveNodeNum[n]].qdot.x = 0.0;
    if (nodes[liveNodeNum[n]].yfixed) nodes[liveNodeNum[n]].qdot.y = 0.0;
    // nodes[liveNodeNum[n]].qdot = nodes[liveNodeNum[n]].f;
    nodes[liveNodeNum[n]].q += nodes[liveNodeNum[n]].qdot * dt;
  }

  // 4) ==== Update velocities of the MPs (FLIP/PIC blending)
  OneStep::updateMPVelocity(MPM);

  // 4') ==== Update positions of the MPs
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    MP[p].prev_pos = MP[p].pos;
    double invmass;
    for (size_t r = 0; r < element::nbNodes; r++) {
      if (nodes[I[r]].mass > tolmass) {
        invmass = 1.0 / nodes[I[r]].mass;
        MP[p].pos += dt * MP[p].N[r] * nodes[I[r]].q * invmass;
      }
    }
  }

  // 4a) ==== End-of-step nodal velocities
  //
  // This is what makes the scheme an 'update stress LAST': the strain increment
  // has to be built on the velocity field AFTER the momenta have been updated
  // in 3). Reading here the velocities computed in 1a), i.e. those of the
  // beginning of the step, while the internal forces of 2) come from the stress
  // of the previous step, is a combination known to be unstable -- and it was
  // indeed enough to make a simple elastic column diverge and throw a Material
  // Point out of the grid. ModifiedLagrangian does the same refresh before its
  // own stress update; UpdateStressFirst does not need it, since it computes
  // the stress before the forces.
  for (size_t n = 0; n < liveNodeNum.size(); n++) {
    if (nodes[liveNodeNum[n]].mass > tolmass)
      nodes[liveNodeNum[n]].vel = nodes[liveNodeNum[n]].q / nodes[liveNodeNum[n]].mass;
    else
      nodes[liveNodeNum[n]].vel.reset();
  }

  // 4b) ==== Deformation gradient and Volume (C)
  // Moved down from before the force computation: it reads nodes[].vel through
  // updateVelocityGradient and has to see the refreshed field too.
  MPM.updateTransformationGradient();
  for (size_t p = 0; p < MP.size(); p++) {
    MP[p].vol = MP[p].F.det() * MP[p].vol0;
  }
  OneStep::updateDensityFromVolume(MPM);

  // 5) ==== Update strain and stress (CHCL models included)
  OneStep::updateStrainAndStress(MPM);

  // ==== Update the corner positions of the MPs
  for (size_t p = 0; p < MP.size(); p++) {
    MP[p].updateCornersFromF();
  }
	
  return 0;
}
