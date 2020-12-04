#include "ModifiedLagrangian.hpp"

#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>
#include <Obstacles/Obstacle.hpp>
#include <blender_Obstacles/blender_Obstacle.hpp>
#include <ShapeFunctions/ShapeFunction.hpp>
#include <Core/Node.hpp>
#include <Core/Element.hpp>
#include <ConstitutiveModels/ConstitutiveModel.hpp>
// #include <Spies/Spy.hpp>

#include <factory.hpp>
static Registrar<OneStep, ModifiedLagrangian> registrar("ModifiedLagrangian");



void ModifiedLagrangian::advanceOneStep (MPMbox & MPM)
{
	// Defining aliases
	std::vector<node> &nodes = MPM.nodes;
	std::vector<int> &liveNodeNum = MPM.liveNodeNum;
	std::vector<element>  & Elem = MPM.Elem;
	std::vector<MaterialPoint>  &MP = MPM.MP;
	std::vector<Obstacle*>  &Obstacles = MPM.Obstacles;  // List of rigid obstacles
	std::vector<blender_Obstacle*>  &blender_Obstacles = MPM.blender_Obstacles;  // List of rigid obstacles
	// std::vector<Spy*>    &Spies = MPM.Spies;
	// double &t = MPM.t;
	double &dt = MPM.dt;
	double &tolmass = MPM.tolmass;
	int &step = MPM.step;
	vec3r &gravity = MPM.gravity;

	//End of aliases


	//Explained in Issam (Thesis)
	//Algorithm written in my notebook2
	//Mass and density are updated in this onestep


	if (step == 0) std::cout<<"Running Modified Lagrangian1"<<std::endl;
	int *I; // use as node index

	// ==== Discard previous grid
	for (size_t n = 0 ; n < liveNodeNum.size() ; n++) {

		nodes[liveNodeNum[n]].mass = 0.0;
		nodes[liveNodeNum[n]].q.reset();
		nodes[liveNodeNum[n]].qdot.reset();
		nodes[liveNodeNum[n]].f.reset();
		nodes[liveNodeNum[n]].fb.reset();
		nodes[liveNodeNum[n]].vel.reset();
	}

	MPM.number_MP = MP.size();
	// ==== Reset the resultant forces on MPs and velGrad
	for (size_t p = 0 ; p < MP.size() ; p++) {
		MP[p].f.reset();
	}
	//Part A) moving obstacles
	// ==== Delete computed resultants (force and moment) of rigid obstacles
	for (size_t o = 0 ; o < Obstacles.size() ; ++o) {
		Obstacles[o]->force.reset();
		Obstacles[o]->mom = 0.0;
		Obstacles[o]->acc = gravity;
		Obstacles[o]->arot = 0.0;
	}

	for (size_t o = 0 ; o < blender_Obstacles.size() ; ++o) {
		blender_Obstacles[o]->force.reset();
		blender_Obstacles[o]->mom = 0.0;
		blender_Obstacles[o]->acc = gravity;
		blender_Obstacles[o]->arot = 0.0;
	}

	// ==== Compute interpolation values
	for (size_t p = 0 ; p < MP.size() ; p++) {
		MPM.shapeFunction->computeInterpolationValues(MPM, p);
	}

	// ==== Update Vector of node indices
	std::set<int> sortedLive;
	for (size_t p = 0; p < MP.size(); p ++) {
		I = &(Elem[MP[p].e].I[0]);
		for (int r = 0 ; r < 8 ; r++) {
			sortedLive.insert(I[r]);
		}
	}
	liveNodeNum.clear();
	std::copy(sortedLive.begin(), sortedLive.end(), std::back_inserter(liveNodeNum));

	//weightIncrement();  // gradually increments the weight, given a number of time steps

	//Part B) moving obstacles
	// ==== Move the rigid obstacles according to their mode of driving
	double dt_2 = 0.5 * dt;
	double dt2_2 = dt_2 * dt;
	for (size_t o = 0 ; o < Obstacles.size() ; ++o) {
		if (Obstacles[o]->isFree) {
            //if (t >= timeNumericalDissipation) {
                Obstacles[o]->pos += Obstacles[o]->vel * dt + Obstacles[o]->acc * dt2_2;
                Obstacles[o]->vel += Obstacles[o]->acc * dt_2;
                Obstacles[o]->rot += Obstacles[o]->vrot * dt + Obstacles[o]->arot * dt2_2;
                Obstacles[o]->vrot += Obstacles[o]->arot * dt_2;
            //}
		}
		else { // velocity is imposed (rotations are supposed blocked)
			Obstacles[o]->pos += Obstacles[o]->vel * dt;
		}
	}
	//FIXME: same code but for blender_obstacles
	for (size_t o = 0 ; o < blender_Obstacles.size() ; ++o) {
		// std::cout<<blender_Obstacles[o]->obstacleName<<std::endl;
		if (blender_Obstacles[o]->isFree) {
            //if (t >= timeNumericalDissipation) {
                blender_Obstacles[o]->pos += blender_Obstacles[o]->vel * dt + blender_Obstacles[o]->acc * dt2_2;
                blender_Obstacles[o]->vel += blender_Obstacles[o]->acc * dt_2;
                blender_Obstacles[o]->rot += blender_Obstacles[o]->vrot * dt + blender_Obstacles[o]->arot * dt2_2;
                blender_Obstacles[o]->vrot += blender_Obstacles[o]->arot * dt_2;
            //}
		}
		else { // velocity is imposed (rotations are supposed blocked)
			blender_Obstacles[o]->pos += blender_Obstacles[o]->vel * dt;
		}
	}
	//if (step % 1000 == 0) std::cout<<"pos inside MPMbox class"<<std::endl<<Obstacles[0]->pos<<std::endl;

	// 0) ==== Getting material point mass (something we were not doing before)
	for (size_t p = 0 ; p < MP.size() ; p++) {
		MP[p].mass = MP[p].density * MP[p].vol;
	}

	// 1) ==== Initialize grid state (mass and momentum)
	for (size_t p = 0 ; p < MP.size() ; p++) {
		I = &(Elem[MP[p].e].I[0]);

		for (int r = 0 ; r < 8 ; r++) {
			// Nodal mass
			nodes[I[r]].mass += MP[p].N[r] * MP[p].mass;
			// Nodal momentum
			//nodes[I[r]].q += MP[p].N[r] * MP[p].mass * MP[p].vel;  //he uses it but apparently its not necessary since we then replace it with the vel from the MP

			// Blocked DOFs (at nodes)
			if (nodes[I[r]].xfixed) { nodes[I[r]].q.x = 0.0; }
			if (nodes[I[r]].yfixed) { nodes[I[r]].q.y = 0.0; }
		}
	}


	// 2) ==== Compute internal and external forces
	for (size_t p = 0 ; p < MP.size() ; p++) {
		I = &(Elem[MP[p].e].I[0]);

		for (int r = 0 ; r < 8 ; r++) {
			// Internal forces
			nodes[I[r]].f += -MP[p].vol * (MP[p].stress * MP[p].gradN[r]);
			// External forces (gravity)
			nodes[I[r]].f += MP[p].mass * gravity * MP[p].N[r];
		}
	}

	// 2b) ==== Boundary Conditions
	MPM.boundaryConditions();
	MPM.blenderBoundaryConditions();

	//Part C) moving obstacles
	for (size_t o = 0 ; o < Obstacles.size() ; ++o) {
		if (Obstacles[o]->isFree) {
			Obstacles[o]->acc = Obstacles[o]->force / Obstacles[o]->mass;
			Obstacles[o]->vel += Obstacles[o]->acc * dt_2;
			Obstacles[o]->arot = Obstacles[o]->mom / Obstacles[o]->I;
			Obstacles[o]->vrot += Obstacles[o]->arot * dt_2;
		}
	}
	//FIXME: same but for blender_Obstacles
	for (size_t o = 0 ; o < blender_Obstacles.size() ; ++o) {
		if (blender_Obstacles[o]->isFree) {
			blender_Obstacles[o]->acc = blender_Obstacles[o]->force / blender_Obstacles[o]->mass;
			blender_Obstacles[o]->vel += blender_Obstacles[o]->acc * dt_2;
			blender_Obstacles[o]->arot = blender_Obstacles[o]->mom / blender_Obstacles[o]->I;
			blender_Obstacles[o]->vrot += blender_Obstacles[o]->arot * dt_2;
		}
	}

	for (size_t p = 0 ; p < MP.size() ; p++) {
		//sending BC forces to the nodes
		I = &(Elem[MP[p].e].I[0]);
		for (int r = 0 ; r < 8 ; r++) {
			nodes[I[r]].fb += MP[p].f * MP[p].N[r];
		}
	}

	// 3) ==== Compute rate of momentum and update nodes
	for (size_t n = 0 ; n < liveNodeNum.size() ; n++) {
		// sum of boundary and volume forces:
		nodes[liveNodeNum[n]].qdot = nodes[liveNodeNum[n]].fb + nodes[liveNodeNum[n]].f;
		if ( !(nodes[liveNodeNum[n]].xfixed) ) nodes[liveNodeNum[n]].q.x += nodes[liveNodeNum[n]].qdot.x * dt;
		else nodes[liveNodeNum[n]].qdot.x = 0.0;
		if ( !(nodes[liveNodeNum[n]].yfixed) ) nodes[liveNodeNum[n]].q.y += nodes[liveNodeNum[n]].qdot.y * dt;
		else nodes[liveNodeNum[n]].qdot.y = 0.0;
		//nodes[liveNodeNum[n]].q += nodes[liveNodeNum[n]].qdot*dt;
	}

	// 3a) ==== Calculate velocity in MP (to then update q). sort of smoothing?
	for (size_t p = 0 ; p < MP.size() ; p++) {
		I = &(Elem[MP[p].e].I[0]);

		double invmass;
		for (int r = 0 ; r < 8 ; r++) {
			if (nodes[I[r]].mass > tolmass) {
                invmass = 1.0 / nodes[I[r]].mass;
                MP[p].vel += dt * MP[p].N[r]*nodes[I[r]].qdot * invmass;
			}
		}
	}

	// 3b) ==== Calculate updated momentum in nodes
	for (size_t p = 0 ; p < MP.size() ; p++) {
		I = &(Elem[MP[p].e].I[0]);

		for (int r = 0 ; r < 8 ; r++) {
			// Nodal momentum
			nodes[I[r]].q += MP[p].N[r] * MP[p].mass * MP[p].vel;
		}
	}

	// 3c) ==== Nodal velocities  (A)
	for (size_t n = 0 ; n < liveNodeNum.size() ; n++) {
		if (nodes[liveNodeNum[n]].mass > tolmass) {
			nodes[liveNodeNum[n]].vel = nodes[liveNodeNum[n]].q / nodes[liveNodeNum[n]].mass;
		}
		else {
			nodes[liveNodeNum[n]].vel.reset();
		}
	}

	// 3d) ====Deformation gradient (C)
	//updateTransformationGradient();  //not added yet

	// 4) ==== Update strain and stress
	for (size_t p = 0 ; p < MP.size() ; p++) {
		MP[p].constitutiveModel->updateStrainAndStress(MPM, p);
	}

	// 4a) ====Update Volume and density
	for (size_t p = 0; p < MP.size(); p++) {
		double volumetricdStrain = MP[p].deltaStrain.xx + MP[p].deltaStrain.yy;
		MP[p].vol *= (1 + volumetricdStrain);
		MP[p].density /= (1 + volumetricdStrain);
	}

	// 5) ==== Update positions
	for (size_t p = 0 ; p < MP.size() ; p++) {
		I = &(Elem[MP[p].e].I[0]);
		MP[p].prev_pos = MP[p].pos;
		double invmass;
		for (int r = 0 ; r < 8 ; r++) {
			if (nodes[I[r]].mass > tolmass) {
                invmass = 1.0 / nodes[I[r]].mass;
                //MP[p].vel += dt * MP[p].N[r]*nodes[I[r]].qdot * invmass;
                MP[p].pos += dt * MP[p].N[r]*nodes[I[r]].q*invmass;
			}
		}
	}

	// ==== Update the corner positions of the MPs
	// for (size_t p = 0 ; p < MP.size() ; p++) {
	// 	MP[p].updateCornersFromF();
	// }

	// // ==== Split MPs
	// if (splitting) adaptativeRefinement();
	// if (splittingMore) adaptativeRefinementMore();

	// // ==== Execute the spies
	// for (size_t s = 0 ; s < Spies.size() ; ++s) {
	// 	if (step % (Spies[s]->nstep) == 0) Spies[s]->exec();
	// 	if (step % (Spies[s]->nrec) == 0) Spies[s]->record();
	// }

	//Start of smoothVelocity
	for (size_t p = 0 ; p < MP.size() ; p++) {
		I = &(Elem[MP[p].e].I[0]);
		for (int r = 0 ; r < 8 ; r++) {
			nodes[I[r]].vel += MP[p].N[r] * MP[p].mass * MP[p].vel / nodes[I[r]].mass;
		}
	}
	//std::vector<vec2r> smoothVelocity(MP.size());
	// Material-point velocity (Smoothed)

	for (size_t p = 0 ; p < MP.size() ; p++) {
		MP[p].smoothVel.reset();
		I = &(Elem[MP[p].e].I[0]);
		for (int r = 0 ; r < 8 ; r++) {
			MP[p].smoothVel += nodes[I[r]].vel * MP[p].N[r];
		}
	}
	//end of smoothVelocity

	// ==== Update time
	// t += dt;

}
