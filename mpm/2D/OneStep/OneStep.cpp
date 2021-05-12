#include "OneStep.hpp"

OneStep::~OneStep() {}

void OneStep::plug(MPMbox* Box) { box = Box; }

void OneStep::resetDEM(Obstacle* obst, vec2r gravity) {
  // ==== Delete computed resultants (force and moment) of rigid obstacles
  obst->force.reset();
  obst->mom = 0.0;
  obst->acc = gravity;
  obst->arot = 0.0;
}

void OneStep::moveDEM1(Obstacle* obst, double dt, bool activeNumericalDissipation) {
  // ==== Move the rigid obstacles according to their mode of driving
  double dt_2 = 0.5 * dt;
  double dt2_2 = dt_2 * dt;
  if (obst->isFree) {
    if (activeNumericalDissipation == false) {
      obst->pos += obst->vel * dt + obst->acc * dt2_2;
      obst->vel += obst->acc * dt_2;
      obst->rot += obst->vrot * dt + obst->arot * dt2_2;
      obst->vrot += obst->arot * dt_2;
    }
  } else {  // velocity is imposed (rotations are supposed blocked)
    obst->pos += obst->vel * dt;
  }
}
void OneStep::moveDEM2(Obstacle* obst, double dt) {
  double dt_2 = 0.5 * dt;
  if (obst->isFree) {
    obst->acc = obst->force / obst->mass;
    obst->vel += obst->acc * dt_2;
    obst->arot = obst->mom / obst->I;
    obst->vrot += obst->arot * dt_2;
  }
}

vec2r OneStep::numericalDissipation(vec2r velMP, vec2r forceMP) {
  // because there is no sign function...

  int sign = 0;
  double product = forceMP * velMP;
  if (product >= 0)
    sign = 1;
  else
    sign = -1;

  // the value shouldnt be too high cuz it will freeze the material and
  // when its released the waves will reappear
  vec2r deltaF = -0.5 * sign * forceMP;
  forceMP += deltaF;

  return forceMP;
}

// FIXME: THis boundaryCondition is deprecated. New class called BoundaryForceLaw
// TODO: should be removed at some point
// void OneStep::boundaryConditions(std::vector<MaterialPoint> &MP, std::vector<Obstacle*> &Obstacles,
// 	DataTable dataTable, size_t id_kn, size_t id_kt, size_t id_en2, size_t id_mu, double dt) {
// 	//TODO: Make boundaries independent (different friction for example) Use groups (somehow already implemented)
// 	double kn, kt, en2, mu;
// 	size_t g1, g2;
// 	for (size_t o = 0 ; o < Obstacles.size() ; ++o) {
// 		for (size_t nn = 0 ; nn < Obstacles[o]->Neighbors.size() ; ++nn) {
// 			size_t pn = Obstacles[o]->Neighbors[nn].PointNumber;
// 			double dn;
// 			int corner = Obstacles[o]->touch(MP[pn], dn);
//
// 			// if it returns -1 there is no contact, else it returns a positive number
// 			if ( corner >= 0 ) { // Check if there is contact
// 				g1 = (size_t)(MP[pn].groupNb);
// 				g2 = Obstacles[o]->group;
// 				kn = dataTable.get(id_kn, g1, g2);
// 				kt = dataTable.get(id_kt, g1, g2);
// 				en2 = dataTable.get(id_en2, g1, g2);
// 				mu = dataTable.get(id_mu, g1, g2);
//
// 				vec2r N, T;
// 				Obstacles[o]->getContactFrame(MP[pn], N, T);
//
// 				// === Normal force
// 				if (dn < 0.0) { // overlapping
// 					double delta_dn = dn - Obstacles[o]->Neighbors[nn].dn;
// 					if (delta_dn > 0.0) {      // Unloading
// 						Obstacles[o]->Neighbors[nn].fn = -kn * en2 * dn;
// 					}
// 					else if (delta_dn < 0.0) { // Loading
// 						Obstacles[o]->Neighbors[nn].fn += -kn * delta_dn;
// 					}
// 					else {
// 						Obstacles[o]->Neighbors[nn].fn = -kn * dn; // First time loading
// 					}
// 				}
//
// 				Obstacles[o]->Neighbors[nn].dn = dn;
//
// 				// === Friction force
// 				vec2r lever = MP[pn].pos - Obstacles[o]->pos;
// 				vec2r zCrossLever(-lever.y, lever.x);
// 				double delta_dt = ((MP[pn].pos - MP[pn].prev_pos) - dt*(Obstacles[o]->vel + Obstacles[o]->vrot *
// zCrossLever)) * T;
//
// 				Obstacles[o]->Neighbors[nn].ft += -kt * delta_dt;
// 				double threshold = mu * Obstacles[o]->Neighbors[nn].fn;
//
// 				if (Obstacles[o]->Neighbors[nn].ft > threshold)
// 					Obstacles[o]->Neighbors[nn].ft = threshold;
// 				if (Obstacles[o]->Neighbors[nn].ft < -threshold)
// 					Obstacles[o]->Neighbors[nn].ft = -threshold;
//
// 				// if(Obstacles[o]->group == 0){
// 				// 	std::cout << "ratio: "
// <<Obstacles[o]->Neighbors[nn].ft/Obstacles[o]->Neighbors[nn].fn;
// 				// 	// std::cout << "mu: " <<mu<< '\n';
// 				// 	std::cout << "   Tang Force: " <<Obstacles[o]->Neighbors[nn].ft;
// 				// 	std::cout << "   Normal Force: " <<Obstacles[o]->Neighbors[nn].fn;
// 				// 	std::cout << "   dn: " <<dn<<std::endl;
// 				// }
// 				// === Resultant force
// 				vec2r f = Obstacles[o]->Neighbors[nn].fn * N + Obstacles[o]->Neighbors[nn].ft * T;
// 				MP[pn].contactf.x = f.x * -1;  //just to show the contact forces in the vtk
// 				MP[pn].contactf.y = f.y * -1;
// 				//std::cout<<f<<std::endl;
// 				MP[pn].f += f;
// 				Obstacles[o]->force -= f;
//
// 				// === Resultant moment (only on the obstacle)
// 				Obstacles[o]->mom += cross(lever, -f);
// 			}
// 			else {
// 				Obstacles[o]->Neighbors[nn].dn = 0.0;
// 				Obstacles[o]->Neighbors[nn].dt = 0.0;
// 				Obstacles[o]->Neighbors[nn].fn = 0.0;
// 				Obstacles[o]->Neighbors[nn].ft = 0.0;
// 				MP[pn].contactf.reset();
// 			} // end if else touch
// 		} // end for-loop over neighbors nn
// 	} // end for-loop over obstacles o
//
// }

//********************************************************************
// OLD BOUNDARY CONDITION WITH CORNERS (not adjusted to OneStep)
/*
void MPMbox::boundaryConditions()
{
        //TODO: Make boundaries independent (different friction for example) Use groups (somehow already implemented)
        double kn, kt, en2, mu;
        size_t g1, g2;

        for (size_t o = 0 ; o < Obstacles.size() ; ++o) {
                for (size_t nn = 0 ; nn < Obstacles[o]->Neighbors.size() ; ++nn) {
                        size_t pn = Obstacles[o]->Neighbors[nn].PointNumber;
                        double dn;
                        int corner = Obstacles[o]->touch(MP[pn], dn);
                        if ( corner >= 0 ) { // Check if there is contact

                                g1 = (size_t)(MP[pn].groupNb);
                                g2 = Obstacles[o]->group;
                                kn = dataTable.get(id_kn, g1, g2);
                                kt = dataTable.get(id_kt, g1, g2);
                                en2 = dataTable.get(id_en2, g1, g2);
                                mu = dataTable.get(id_mu, g1, g2);

                                vec2r N, T;
                                Obstacles[o]->getContactFrame(MP[pn], N, T);

                                // === Normal force
                                if (dn < 0.0) { // overlapping
                                        double delta_dn = dn - Obstacles[o]->Neighbors[nn].dn;
                                        if (delta_dn > 0.0) {      // Unloading
                                                Obstacles[o]->Neighbors[nn].fn = -kn * en2 * dn;
                                        }
                                        else if (delta_dn < 0.0) { // Loading
                                                Obstacles[o]->Neighbors[nn].fn += -kn * delta_dn;
                                        }
                                        else {
                                                Obstacles[o]->Neighbors[nn].fn = -kn * dn; // First time loading
                                        }
                                }

                                Obstacles[o]->Neighbors[nn].dn = dn;

                                // === Friction force
                                vec2r lever = MP[pn].corner[corner] - Obstacles[o]->pos;
                                vec2r zCrossLever(-lever.y, lever.x);
                                double delta_dt = ((MP[pn].pos - MP[pn].prev_pos) - dt*(Obstacles[o]->vel +
Obstacles[o]->vrot * zCrossLever)) * T;

                                Obstacles[o]->Neighbors[nn].ft += -kt * delta_dt;
                                double threshold = mu * Obstacles[o]->Neighbors[nn].fn;

                                if (Obstacles[o]->Neighbors[nn].ft > threshold)
                                        Obstacles[o]->Neighbors[nn].ft = threshold;
                                if (Obstacles[o]->Neighbors[nn].ft < -threshold)
                                        Obstacles[o]->Neighbors[nn].ft = -threshold;

                                // === Resultant force
                                vec2r f = Obstacles[o]->Neighbors[nn].fn * N + Obstacles[o]->Neighbors[nn].ft * T;
                                MP[pn].contactf.x = f.x * -1;  //just to show the contact forces in the vtk
                                MP[pn].contactf.y = f.y * -1;
                                //std::cout<<f<<std::endl;
                                MP[pn].f += f;
                                Obstacles[o]->force -= f;

                                // === Resultant moment (only on the obstacle)
                                Obstacles[o]->mom += cross(lever, -f);
                        }
                        else {
                                Obstacles[o]->Neighbors[nn].dn = 0.0;
                                Obstacles[o]->Neighbors[nn].dt = 0.0;
                                Obstacles[o]->Neighbors[nn].fn = 0.0;
                                Obstacles[o]->Neighbors[nn].ft = 0.0;
                                MP[pn].contactf.reset();
                        } // end if else touch
                } // end for-loop over neighbors nn
        } // end for-loop over obstacles o
}
*/
