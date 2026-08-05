#include "OneStep.hpp"

#include "ConstitutiveModels/ConstitutiveModel.hpp"
#include "Core/Element.hpp"
#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"
#include "Core/Node.hpp"

OneStep::~OneStep() {}

//
// Update the velocity of the Material Points from the nodal quantities.
//
// The raw MPM update is a FLIP one: the point keeps its own velocity and
// receives the increment carried by the grid,
//
//   v_p <- v_p + sum_r N_r dt qdot_r / m_r
//
// which conserves the fine-grained fluctuations, hence very little numerical
// dissipation -- and a fair amount of noise. The PIC update, on the contrary,
// reads the velocity straight from the grid,
//
//   v_p <- sum_r N_r q_r / m_r
//
// which filters everything, at the price of a strong artificial damping. What
// is used here is the usual blend of the two, with ratioFLIP = 1 - ratioPIC:
// pure FLIP for ratioFLIP = 1, pure PIC for ratioFLIP = 0.
//
// This is what the 'enablePIC' and 'disablePIC' keywords, and the
// PICDissipation and PICDissipationByPIC schedulers, act upon.
//
void OneStep::updateMPVelocity(MPMbox& MPM) {
  START_TIMER("updateMPVelocity");

  std::vector<node>& nodes = MPM.nodes;
  std::vector<element>& Elem = MPM.Elem;
  std::vector<MaterialPoint>& MP = MPM.MP;
  const double dt = MPM.dt;

  for (size_t p = 0; p < MP.size(); p++) {
    size_t* I = &(Elem[MP[p].e].I[0]);

    if (MPM.activePIC == true) {
      vec2r PICvelocity;
      for (size_t r = 0; r < element::nbNodes; r++) {
        if (nodes[I[r]].mass > MPM.tolmass) {
          const double invmass = 1.0 / nodes[I[r]].mass;
          PICvelocity += MP[p].N[r] * nodes[I[r]].q * invmass;
          MP[p].vel += MP[p].N[r] * dt * nodes[I[r]].qdot * invmass;
        }
      }
      MP[p].vel = MPM.ratioFLIP * MP[p].vel + (1.0 - MPM.ratioFLIP) * PICvelocity;
    } else {
      for (size_t r = 0; r < element::nbNodes; r++) {
        if (nodes[I[r]].mass > MPM.tolmass) {
          const double invmass = 1.0 / nodes[I[r]].mass;
          MP[p].vel += MP[p].N[r] * dt * nodes[I[r]].qdot * invmass;
        }
      }
    }
  }
}

//
// Call the constitutive model of every Material Point.
//
// When at least one point carries a computationally homogenised law (CHCL), the
// points are sorted into two lists before being run in parallel: a DEM cell
// costs orders of magnitude more than a closed-form law, so mixing them in a
// single OpenMP loop would leave most threads waiting.
//
void OneStep::updateStrainAndStress(MPMbox& MPM) {
  START_TIMER("updateStrainAndStress");

  std::vector<MaterialPoint>& MP = MPM.MP;

  if (MPM.CHCL.hasDoubleScale == false) {
    for (size_t p = 0; p < MP.size(); p++) { MP[p].constitutiveModel->updateStrainAndStress(MPM, p); }
    return;
  }

  std::vector<size_t> simpleScale;
  std::vector<size_t> doubleScale;
  for (size_t p = 0; p < MP.size(); p++) {
    if (MP[p].isDoubleScale) {
      doubleScale.push_back(p);
    } else {
      simpleScale.push_back(p);
    }
  }

  // Single-scale MPs
#pragma omp parallel for default(shared)
  for (size_t q = 0; q < simpleScale.size(); q++) {
    MP[simpleScale[q]].constitutiveModel->updateStrainAndStress(MPM, simpleScale[q]);
  }

  // Two-scale MPs
#pragma omp parallel for default(shared)
  for (size_t q = 0; q < doubleScale.size(); q++) {
    MP[doubleScale[q]].constitutiveModel->updateStrainAndStress(MPM, doubleScale[q]);
    Logger::trace("Stress for MP #{} = xx={} / xy={} / yx={} / yy={}", doubleScale[q], MP[doubleScale[q]].stress.xx,
                  MP[doubleScale[q]].stress.xy, MP[doubleScale[q]].stress.yx, MP[doubleScale[q]].stress.yy);
  }
}

//
// Keep the density consistent with the current volume.
//
// The mass of a Material Point does not change, so the density has to follow
// the volume. Deriving it from mass / vol rather than applying a multiplicative
// correction keeps 'mass = vol * density' exact, however long the computation
// runs. The density is what the viewer displays and what feeds rhoMin in
// MPMbox::convergenceConditions; UpdateStressFirst and UpdateStressLast used to
// leave it at its initial value for ever.
//
void OneStep::updateDensityFromVolume(MPMbox& MPM) {
  std::vector<MaterialPoint>& MP = MPM.MP;
  for (size_t p = 0; p < MP.size(); p++) {
    if (MP[p].vol > 0.0) { MP[p].density = MP[p].mass / MP[p].vol; }
  }
}

void OneStep::resetDEM(Obstacle* obst, vec2r gravity) {
  // ==== Delete computed resultants (force and moment) of rigid obstacles
  obst->force.reset();
  obst->mom = 0.0;
  obst->acc = gravity;
  obst->arot = 0.0;
}

void OneStep::moveDEM1(Obstacle* obst, double dt) {
  // ==== Move the rigid obstacles according to their mode of driving
  if (obst->isFree) {
		
    double dt_2 = 0.5 * dt;
    double dt2_2 = dt_2 * dt;
    obst->pos += obst->vel * dt + obst->acc * dt2_2;
    obst->vel += obst->acc * dt_2;
    obst->rot += obst->vrot * dt + obst->arot * dt2_2;
    obst->vrot += obst->arot * dt_2;
		
  } else {  // velocity is imposed (rotations are supposed blocked)
		
    obst->pos += obst->vel * dt;
  
	}
}

void OneStep::moveDEM2(Obstacle* obst, double dt) {
  if (obst->isFree) {
		
    double dt_2 = 0.5 * dt;
    obst->acc = obst->force / obst->mass;
    obst->vel += obst->acc * dt_2;
    obst->arot = obst->mom / obst->I;
    obst->vrot += obst->arot * dt_2;
		
  }
}

vec2r OneStep::numericalDissipation(vec2r velMP, vec2r forceMP, double coefficient) {
  vec2r vecSigned;
  vecSigned.x = (forceMP.x * velMP.x > 0.0) ? (1.0 - coefficient) : (1.0 + coefficient);
  vecSigned.y = (forceMP.y * velMP.y > 0.0) ? (1.0 - coefficient) : (1.0 + coefficient);
  return vecSigned;
}

/*
vec2r OneStep::numericalDissipation(vec2r velMP, vec2r forceMP, double cundall) {
  double factor;
  double factorMinus = 1.0 - cundall;
  double factorPlus = 1.0 + cundall;
  factor = (forceMP.x * velMP.x > 0.0) ? factorMinus : factorPlus;
  forceMP.x *= factor;
  factor = (forceMP.y * velMP.y > 0.0) ? factorMinus : factorPlus;
  std::cout << "cundall factor" << factor << std::endl;
  forceMP.y *= factor;
  return forceMP;
}
*/
