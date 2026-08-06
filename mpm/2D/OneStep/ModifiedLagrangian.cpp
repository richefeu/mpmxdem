// Notice that only this integration scheme can be employed for double scale usage

#include "ModifiedLagrangian.hpp"

#include "ConstitutiveModels/ConstitutiveModel.hpp"
#include "Core/Element.hpp"
#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"
#include "Core/Node.hpp"
#include "Obstacles/Obstacle.hpp"
#include "ShapeFunctions/ShapeFunction.hpp"
#include "Spies/Spy.hpp"

#include "PBC3D.hpp"

std::string ModifiedLagrangian::getRegistrationName() { return std::string("ModifiedLagrangian"); }

// 
// One step of the Modified Lagrangian or Modified Update Stress Last (MUSL) integration scheme.
// 
// Updates the positions, velocities, deformation gradients, strains, and stresses of all MaterialPoints.
// 
int ModifiedLagrangian::advanceOneStep(MPMbox& MPM) {
  START_TIMER("MUSL step");

  // Defining aliases ======================================
  std::vector<node>& nodes = MPM.nodes;
  std::vector<size_t>& liveNodeNum = MPM.liveNodeNum;
  std::vector<element>& Elem = MPM.Elem;
  std::vector<MaterialPoint>& MP = MPM.MP;
  std::vector<Obstacle*>& Obstacles = MPM.Obstacles;
  double& dt = MPM.dt;
  // End of aliases ========================================

  size_t* I;  // use as node index

  // ==== Discard previous grid
  {
    START_TIMER("grid reset");
  for (size_t n = 0; n < liveNodeNum.size(); n++) {
    nodes[liveNodeNum[n]].mass = 0.0;
    nodes[liveNodeNum[n]].outOfPlaneStress = 0.0;
    nodes[liveNodeNum[n]].q.reset();
    nodes[liveNodeNum[n]].qdot.reset();
    nodes[liveNodeNum[n]].f.reset();
    nodes[liveNodeNum[n]].fb.reset();
    nodes[liveNodeNum[n]].vel.reset();
  }

  MPM.number_MP_before_any_split = MPM.MP.size();

  // shapeN / shapeGradN suivent le nombre de points
  MPM.resizeMPArrays();

  // ==== Reset the resultant forces of MPs
  // (velGrad is cleared by MPMbox::updateVelocityGradient, so that no
  //  integration scheme can forget it)
  for (size_t p = 0; p < MP.size(); p++) {
    MP[p].f.reset();
  }
  }

  // ==== Delete computed resultants (force and moment) of rigid obstacles
  for (size_t o = 0; o < Obstacles.size(); ++o) {
    OneStep::resetDEM(Obstacles[o], MPM.gravity);
  }

  // ==== Compute interpolation values
  {
    START_TIMER("shape functions");
    for (size_t p = 0; p < MPM.MP.size(); p++) {
      MPM.shapeFunction->computeInterpolationValues(MPM, p);
    }
  }

  // ==== Update Vector of node indices
  MPM.updateLiveNodeList();

  // ==== Move the rigid obstacles according to their mode of driving
  for (size_t o = 0; o < Obstacles.size(); ++o) {
    OneStep::moveDEM1(Obstacles[o], dt);
  }

  // ==== Initialize grid state (mass and momentum)
  {
    START_TIMER("P2G mass momentum");
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);

    const double *Np = MPM.N(p);
    for (size_t r = 0; r < element::nbNodes; r++) {
      // Nodal mass
      nodes[I[r]].mass += Np[r] * MP[p].mass;
      nodes[I[r]].outOfPlaneStress += Np[r] * MP[p].outOfPlaneStress;
      nodes[I[r]].q += Np[r] * MP[p].vel * MP[p].mass;

      if (nodes[I[r]].xfixed) {
        nodes[I[r]].q.x = 0.0;
      }
      if (nodes[I[r]].yfixed) {
        nodes[I[r]].q.y = 0.0;
      }
    }
  }
  }

  // ==== Compute internal and external forces
  {
    START_TIMER("P2G internal forces");
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);

    const double *Np = MPM.N(p);
    const vec2r *gNp = MPM.gradN(p);
    for (size_t r = 0; r < element::nbNodes; r++) {
      // Internal forces
      nodes[I[r]].f += -MP[p].vol * (MP[p].stress * gNp[r]);
      // External forces (gravity)
      nodes[I[r]].f += MP[p].mass * MPM.gravity * Np[r];
    }
  }
  }

  // Updating free boundary conditions
  {
    START_TIMER("contact forces");
    for (size_t o = 0; o < Obstacles.size(); ++o) {
      Obstacles[o]->boundaryForceLaw->computeForces(MPM, o);
    }

    for (size_t o = 0; o < Obstacles.size(); ++o) {
      OneStep::moveDEM2(Obstacles[o], dt);
    }

    for (size_t p = 0; p < MP.size(); p++) {
      I = &(Elem[MP[p].e].I[0]);
      const double *Np = MPM.N(p);
      for (size_t r = 0; r < element::nbNodes; r++) {
        nodes[I[r]].fb += MP[p].f * Np[r];
      }
    }
  }

  // ==== Compute rate of momentum and update nodes
  {
    START_TIMER("nodal update");
  for (size_t n = 0; n < liveNodeNum.size(); n++) {
    // sum of boundary and volume forces:
    nodes[liveNodeNum[n]].qdot = nodes[liveNodeNum[n]].fb + nodes[liveNodeNum[n]].f;

    if (nodes[liveNodeNum[n]].xfixed) {
      nodes[liveNodeNum[n]].qdot.x = 0.0;
    }
    if (nodes[liveNodeNum[n]].yfixed) {
      nodes[liveNodeNum[n]].qdot.y = 0.0;
    }
    nodes[liveNodeNum[n]].q += dt * nodes[liveNodeNum[n]].qdot;
  }
  }

  // ==== Calculate velocity in MP (to then update q). sort of smoothing
  OneStep::updateMPVelocity(MPM);

  // ==== We may impose x- or y-velocity of some MP (it will overwrite those just computed)
#if 0
  for (size_t cMP = 0; cMP < MPM.controlledMP.size(); cMP++) {
    if (MPM.controlledMP[cMP].xcontrol == VEL_CONTROL) {
      MP[MPM.controlledMP[cMP].PointNumber].vel.x = MPM.controlledMP[cMP].xvalue;
    }
    if (MPM.controlledMP[cMP].ycontrol == VEL_CONTROL) {
      MP[MPM.controlledMP[cMP].PointNumber].vel.y = MPM.controlledMP[cMP].yvalue;
    }
  }
#endif

  // ==== Calculate updated velocity in nodes to compute deformation
  {
    START_TIMER("P2G velocity remap");
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    double invmass;
    const double *Np = MPM.N(p);
    for (size_t r = 0; r < element::nbNodes; r++) {
      if (nodes[I[r]].mass > MPM.tolmass) {
        invmass = 1.0f / nodes[I[r]].mass;
        nodes[I[r]].vel += invmass * Np[r] * MP[p].vel * MP[p].mass;
      } else {
        nodes[I[r]].vel.reset();
      }
    }
  }
  }

  // ==== Deformation gradient
  MPM.updateTransformationGradient();

  // ==== Update strain and stress
  OneStep::updateStrainAndStress(MPM);

  // ==== Update positions avec le q provisoire
  {
    START_TIMER("G2P position");
  for (size_t p = 0; p < MP.size(); p++) {
    // Same place as in UpdateStressFirst and UpdateStressLast: prev_pos holds
    // the position at the end of the previous step, so that pos - prev_pos is
    // a genuine displacement increment. Without it, frictionalNormalRestitution
    // built its tangential force on the displacement since t = 0, and the Work
    // and EnergyBalance spies summed that same total at every step.
    MP[p].prev_pos = MP[p].pos;
    I = &(Elem[MP[p].e].I[0]);
    double invmass;
    const double *Np = MPM.N(p);
    for (size_t r = 0; r < element::nbNodes; r++) {
      if (nodes[I[r]].mass > MPM.tolmass) {
        invmass = 1.0f / nodes[I[r]].mass;
        MP[p].pos += Np[r] * dt * (nodes[I[r]].q) * invmass;
      }
    }
  }
  }

  // ==== Update Volume and density
  {
    START_TIMER("MP volume corners");
  for (size_t p = 0; p < MP.size(); p++) {
    double volumetricdStrain = MP[p].deltaStrain.xx + MP[p].deltaStrain.yy + MP[p].deltaStrain.det();
    MP[p].vol *= (1.0 + volumetricdStrain);
  }
  OneStep::updateDensityFromVolume(MPM);

  }

  return 0;
}
