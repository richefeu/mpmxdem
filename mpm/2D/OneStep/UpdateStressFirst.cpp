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

//#include <factory.hpp>
//static Registrar<OneStep, UpdateStressFirst> registrar("UpdateStressFirst");

std::string UpdateStressFirst::getRegistrationName() { return std::string("UpdateStressFirst"); }

int UpdateStressFirst::advanceOneStep(MPMbox& MPM) {
  // Defining aliases =============================
  std::vector<node>& nodes = MPM.nodes;
  std::vector<size_t>& liveNodeNum = MPM.liveNodeNum;
  std::vector<element>& Elem = MPM.Elem;
  std::vector<MaterialPoint>& MP = MPM.MP;
  std::vector<Obstacle*>& Obstacles = MPM.Obstacles;
  std::vector<Spy*>& Spies = MPM.Spies;
  double& dt = MPM.dt;
  double& tolmass = MPM.tolmass;
  //int& step = MPM.step;
  vec2r& gravity = MPM.gravity;
  // End of aliases ================================

  if (MPM.step == 0) std::cout << "Running UpdateStressFirst" << std::endl;
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

  MPM.number_MP_before_any_split = MPM.MP.size();
  
  // ==== Reset the resultant forces on MPs and velGrad
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

  // ==== Initialize grid state (mass and momentum)
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

  // ==== Nodal velocities  (A)

  for (size_t n = 0; n < liveNodeNum.size(); n++) {
    if (nodes[liveNodeNum[n]].mass > tolmass)
      nodes[liveNodeNum[n]].vel = nodes[liveNodeNum[n]].q / nodes[liveNodeNum[n]].mass;
    else
      nodes[liveNodeNum[n]].vel.reset();
  }

  // ==== Deformation gradient and Volume (C)
  MPM.updateTransformationGradient();
  for (size_t p = 0; p < MP.size(); p++) {
    MP[p].vol = MP[p].F.det() * MP[p].vol0;
  }

  // ==== Update strain and stress
  for (size_t p = 0; p < MP.size(); p++) {
    MP[p].constitutiveModel->updateStrainAndStress(MPM, p);
  }

  // ==== Compute internal and external forces
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    for (size_t r = 0; r < element::nbNodes; r++) {
      // Internal forces
      nodes[I[r]].f += -MP[p].vol * (MP[p].stress * MP[p].gradN[r]);
      // External forces (gravity)
      nodes[I[r]].f += MP[p].mass * gravity * MP[p].N[r];
    }
  }

  // ==== Boundary Conditions
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

  // ==== Compute rate of momentum and update nodes
  for (size_t n = 0; n < liveNodeNum.size(); n++) {
    // sum of boundary and volume forces:
    nodes[liveNodeNum[n]].qdot = nodes[liveNodeNum[n]].fb + nodes[liveNodeNum[n]].f;
    if (!(nodes[liveNodeNum[n]].xfixed))
      nodes[liveNodeNum[n]].q.x += nodes[liveNodeNum[n]].qdot.x * dt;
    else
      nodes[liveNodeNum[n]].qdot.x = 0.0;
    if (!(nodes[liveNodeNum[n]].yfixed))
      nodes[liveNodeNum[n]].q.y += nodes[liveNodeNum[n]].qdot.y * dt;
    else
      nodes[liveNodeNum[n]].qdot.y = 0.0;

    nodes[liveNodeNum[n]].q += nodes[liveNodeNum[n]].qdot * dt;  // newline! we were not updating the q (21-02-2017)
  }

  // ==== Update positions and velocities of the MPs
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    MP[p].prev_pos = MP[p].pos;
    double invmass;
    for (size_t r = 0; r < element::nbNodes; r++) {
      if (nodes[I[r]].mass > tolmass) {
        invmass = 1.0 / nodes[I[r]].mass;
        MP[p].vel += dt * MP[p].N[r] * nodes[I[r]].qdot * invmass;
        MP[p].pos += dt * MP[p].N[r] * nodes[I[r]].q * invmass;
      }
    }
  }

  // ==== Update the corner positions of the MPs
  for (size_t p = 0; p < MP.size(); p++) {
    MP[p].updateCornersFromF();
  }

  // ==== Split MPs
  if (MPM.splitting) MPM.adaptativeRefinement();
  //if (MPM.splittingMore) MPM.adaptativeRefinementMore();

  // ==== Execute the spies
  for (size_t s = 0; s < Spies.size(); ++s) {
    if (MPM.step % (Spies[s]->nstep) == 0) Spies[s]->exec();
    if (MPM.step % (Spies[s]->nrec) == 0) Spies[s]->record();
  }

  // ==== Update time
  // t += dt;

  return 0;
}
