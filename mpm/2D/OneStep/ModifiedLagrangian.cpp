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

#include "factory.hpp"
static Registrar<OneStep, ModifiedLagrangian> registrar("ModifiedLagrangian");

std::string ModifiedLagrangian::getRegistrationName() { return std::string("ModifiedLagrangian"); }

int ModifiedLagrangian::advanceOneStep(MPMbox& MPM) {
  START_TIMER("MUSCL step");

  // Defining aliases ======================================
  std::vector<node>& nodes = MPM.nodes;
  std::vector<int>& liveNodeNum = MPM.liveNodeNum;
  std::vector<element>& Elem = MPM.Elem;
  std::vector<MaterialPoint>& MP = MPM.MP;
  std::vector<Obstacle*>& Obstacles = MPM.Obstacles;
  std::vector<Spy*>& Spies = MPM.Spies;
  double& dt = MPM.dt;
  // End of aliases ========================================

  int* I;  // use as node index

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

  MPM.number_MP = MPM.MP.size();
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
  std::set<int> sortedLive;
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    for (int r = 0; r < element::nbNodes; r++) {
      sortedLive.insert(I[r]);
    }
  }
  liveNodeNum.clear();
  std::copy(sortedLive.begin(), sortedLive.end(), std::back_inserter(liveNodeNum));

  // ==== Move the rigid obstacles according to their mode of driving
  for (size_t o = 0; o < Obstacles.size(); ++o) {
    OneStep::moveDEM1(Obstacles[o], dt);
  }

  // ==== Getting material point mass (something we were not doing before)
  // FIXME: je ne crois pas qu'on devrait faire ça !!!!!!!!!!!
  for (size_t p = 0; p < MP.size(); p++) {
    MP[p].mass = MP[p].density * MP[p].vol;
  }

  // ==== Initialize grid state (mass and momentum)
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);

    for (int r = 0; r < element::nbNodes; r++) {
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

    for (int r = 0; r < element::nbNodes; r++) {
      // Internal forces
      nodes[I[r]].f += -MP[p].vol * (MP[p].stress * MP[p].gradN[r]);
      // External forces (gravity)
      nodes[I[r]].f += MP[p].mass * MPM.gravity * MP[p].N[r];
    }
  }

  // Updating free boundary conditions
  for (size_t o = 0; o < Obstacles.size(); ++o) {
    Obstacles[o]->boundaryForceLaw->computeForces(MPM, o);
  }
  for (size_t o = 0; o < Obstacles.size(); ++o) {
    OneStep::moveDEM2(Obstacles[o], dt);
  }

  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    for (int r = 0; r < element::nbNodes; r++) {
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

  // ==== Calculate velocity in MP (to then update q). sort of smoothing?
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);

    double invmass;
    vec2r PICvelo;
    PICvelo.reset();

    for (int r = 0; r < element::nbNodes; r++) {
      if (nodes[I[r]].mass > MPM.tolmass) {
        invmass = 1.0f / nodes[I[r]].mass;
        PICvelo += MP[p].N[r] * nodes[I[r]].q * invmass;
        MP[p].vel += (MP[p].N[r] * dt * nodes[I[r]].qdot * invmass);
      }
    }

    if (MPM.activePIC) {
      MP[p].vel = MPM.ratioFLIP * MP[p].vel + (1.0 - MPM.ratioFLIP) * PICvelo;
    }
  }

  // ==== Calculate updated velocity in nodes to compute deformation
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    double invmass;
    for (int r = 0; r < element::nbNodes; r++) {
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
    START_TIMER("updateStrainAndStress");
#pragma omp parallel for default(shared)
    for (size_t p = 0; p < MP.size(); p++) {
      MP[p].constitutiveModel->updateStrainAndStress(MPM, p);
    }
  }
  
  // ==== Update positions avec le q provisoire
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    double invmass;
    for (int r = 0; r < element::nbNodes; r++) {
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
  
  MPM.plannedRemovalObstacle();  // FIXME: move it juste before or juste after advanceOneStep
  
  // ==== Split MPs
  if (MPM.splitting) MPM.adaptativeRefinement();

  // ==== Execute the spies
  for (size_t s = 0; s < Spies.size(); ++s) {
    if ((MPM.step % Spies[s]->nstep) == 0) Spies[s]->exec();
    if ((MPM.step % Spies[s]->nrec) == 0) Spies[s]->record();
  }

  return 0;
}
