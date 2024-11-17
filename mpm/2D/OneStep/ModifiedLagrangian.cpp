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

/**
 * @brief One step of the Modified Lagrangian (MUSL) integration scheme.
 *
 * Updates the positions, velocities, deformation gradients, strains, and stresses of all MaterialPoints.
 *
 * @param MPM the MPMbox data structure containing all the relevant information.
 * @return 0 if the computation was successful, 1 otherwise.
 */
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

  // ==== Reset the resultant forces and velGrad of MPs
  for (size_t p = 0; p < MP.size(); p++) {
    MP[p].f.reset();
    MP[p].velGrad.reset();
  }

  // ==== Delete computed resultants (force and moment) of rigid obstacles
  for (size_t o = 0; o < Obstacles.size(); ++o) {
    OneStep::resetDEM(Obstacles[o], MPM.gravity);
  }

  // ==== Compute interpolation values
  for (size_t p = 0; p < MPM.MP.size(); p++) {
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

  // ==== Initialize grid state (mass and momentum)
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);

    for (size_t r = 0; r < element::nbNodes; r++) {
      // Nodal mass
      nodes[I[r]].mass += MP[p].N[r] * MP[p].mass;
      nodes[I[r]].outOfPlaneStress += MP[p].N[r] * MP[p].outOfPlaneStress;
      nodes[I[r]].q += MP[p].N[r] * MP[p].vel * MP[p].mass;

      if (nodes[I[r]].xfixed) {
        nodes[I[r]].q.x = 0.0;
      }
      if (nodes[I[r]].yfixed) {
        nodes[I[r]].q.y = 0.0;
      }
    }
  }

  // ==== Compute internal and external forces
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);

    for (size_t r = 0; r < element::nbNodes; r++) {
      // Internal forces
      nodes[I[r]].f += -MP[p].vol * (MP[p].stress * MP[p].gradN[r]);
      // External forces (gravity)
      nodes[I[r]].f += MP[p].mass * MPM.gravity * MP[p].N[r];
    }
  }

  // Forces imposed to MP
  // TODO !!!!!!

  // Updating free boundary conditions
  for (size_t o = 0; o < Obstacles.size(); ++o) {
    Obstacles[o]->boundaryForceLaw->computeForces(MPM, o);
  }

  for (size_t o = 0; o < Obstacles.size(); ++o) {
    OneStep::moveDEM2(Obstacles[o], dt);
  }

  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    for (size_t r = 0; r < element::nbNodes; r++) {
      nodes[I[r]].fb += MP[p].f * MP[p].N[r];
    }
  }

  // ==== Compute rate of momentum and update nodes
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

  // ==== Calculate velocity in MP (to then update q). sort of smoothing
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);

    if (MPM.activePIC) {
      double invmass;
      vec2r PICvelo;
      for (size_t r = 0; r < element::nbNodes; r++) {
        if (nodes[I[r]].mass > MPM.tolmass) {
          invmass = 1.0f / nodes[I[r]].mass;
          PICvelo += MP[p].N[r] * nodes[I[r]].q * invmass;
          MP[p].vel += MP[p].N[r] * dt * nodes[I[r]].qdot * invmass;
        }
      }
      MP[p].vel = MPM.ratioFLIP * MP[p].vel + (1.0 - MPM.ratioFLIP) * PICvelo;
    } else {
      double invmass;
      for (size_t r = 0; r < element::nbNodes; r++) {
        if (nodes[I[r]].mass > MPM.tolmass) {
          invmass = 1.0f / nodes[I[r]].mass;
          MP[p].vel += MP[p].N[r] * dt * nodes[I[r]].qdot * invmass;
        }
      }
    }
  }

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
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    double invmass;
    for (size_t r = 0; r < element::nbNodes; r++) {
      if (nodes[I[r]].mass > MPM.tolmass) {
        invmass = 1.0f / nodes[I[r]].mass;
        nodes[I[r]].vel += invmass * MP[p].N[r] * MP[p].vel * MP[p].mass;
      } else {
        nodes[I[r]].vel.reset();
      }
    }
  }

  // ==== Deformation gradient
  MPM.updateTransformationGradient();

  // ==== Update strain and stress
  {
    if (MPM.CHCL.hasDoubleScale == true) {
      START_TIMER("updateStrainAndStress");

      // For parallel computing with openMP, we first identify the MPs that do or do not hold a CHCL
      std::vector<size_t> simpleScaleVector;
      std::vector<size_t> doubleScaleVector;
      for (size_t p = 0; p < MP.size(); p++) {
        if (MP[p].isDoubleScale) {
          doubleScaleVector.push_back(p);
        } else {
          simpleScaleVector.push_back(p);
        }
      }

      // Single-scale MPs
#pragma omp parallel for default(shared)
      for (size_t q = 0; q < simpleScaleVector.size(); q++) {
        MP[simpleScaleVector[q]].constitutiveModel->updateStrainAndStress(MPM, simpleScaleVector[q]);
      }

      // Two-scale MPs
#pragma omp parallel for default(shared)
      for (size_t q = 0; q < doubleScaleVector.size(); q++) {
        MP[doubleScaleVector[q]].constitutiveModel->updateStrainAndStress(MPM, doubleScaleVector[q]);
        Logger::trace("Stress for MP #{} = xx={} / xy={} / yx={} / yy={}", doubleScaleVector[q],
                                      MP[doubleScaleVector[q]].stress.xx, MP[doubleScaleVector[q]].stress.xy,
                                      MP[doubleScaleVector[q]].stress.yx, MP[doubleScaleVector[q]].stress.yy);
      }

    } else {

      // Every MPs are single-scale
      for (size_t p = 0; p < MP.size(); p++) {
        MP[p].constitutiveModel->updateStrainAndStress(MPM, p);
      }
    }
  }

  // ==== Update positions avec le q provisoire
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    double invmass;
    for (size_t r = 0; r < element::nbNodes; r++) {
      if (nodes[I[r]].mass > MPM.tolmass) {
        invmass = 1.0f / nodes[I[r]].mass;
        MP[p].pos += MP[p].N[r] * dt * (nodes[I[r]].q) * invmass;
      }
    }
  }

  // ==== Update Volume and density
  for (size_t p = 0; p < MP.size(); p++) {
    double volumetricdStrain = MP[p].deltaStrain.xx + MP[p].deltaStrain.yy + MP[p].deltaStrain.det();
    MP[p].vol *= (1.0 + volumetricdStrain);
    MP[p].density /= (1.0 + volumetricdStrain);
  }

  return 0;
}
