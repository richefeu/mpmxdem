#include "UpdateStressFirst.hpp"

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
static Registrar<OneStep, UpdateStressFirst> registrar("UpdateStressFirst");



void UpdateStressFirst::advanceOneStep (MPMbox & MPM)
{
	// Defining aliases
	std::vector<node> &nodes = MPM.nodes;
	std::vector<int> &liveNodeNum = MPM.liveNodeNum;
	std::vector<element>  & Elem = MPM.Elem;
	std::vector<MaterialPoint>  &MP = MPM.MP;
	// std::vector<Obstacle*>  &Obstacles = MPM.Obstacles;  // List of rigid obstacles
	// std::vector<blender_Obstacle*>  &blender_Obstacles = MPM.blender_Obstacles;  // List of rigid obstacles
	// std::vector<Spy*>    &Spies = MPM.Spies;
	// double &t = MPM.t;
	double &dt = MPM.dt;
	double &tolmass = MPM.tolmass;
	// int &step = MPM.step;
	vec3r &gravity = MPM.gravity;

	//End of aliases

	using std::cout;
	using std::cin;
	using std::endl;

	int *I;

	// 1) Discard previous grid
	for (size_t n = 0 ; n < liveNodeNum.size() ; n++) {
		nodes[liveNodeNum[n]].mass = 0.0;
		nodes[liveNodeNum[n]].q.reset();
		nodes[liveNodeNum[n]].qdot.reset();
		nodes[liveNodeNum[n]].f.reset();
		nodes[liveNodeNum[n]].fb.reset();
		//nodes[n].stress.reset();
	}

	for (size_t p = 0 ; p < MP.size() ; p++) {
		MP[p].f.reset();
	}

	// 2) Compute interpolation values
	for (size_t p = 0 ; p < MP.size() ; p++) {
		MPM.shapeFunction->computeInterpolationValues(MPM, p);
	}

	// ==== NEW Update Vector of node indices
	std::set<int> sortedLive;
	for (size_t p = 0; p < MP.size(); p ++) {
		I = &(Elem[MP[p].e].I[0]);
		for (int r = 0 ; r < 8 ; r++) { //TODO: 0.1) add element::nbNodes like in 2d before adding new shape functions
			sortedLive.insert(I[r]);
		}
	}
	liveNodeNum.clear();
	std::copy(sortedLive.begin(), sortedLive.end(), std::back_inserter(liveNodeNum));

	// 3) Initialize grid state (mass and momentum)
	for (size_t p = 0 ; p < MP.size() ; p++) {
		I = &(Elem[MP[p].e].I[0]);

		for (int r = 0 ; r < 8 ; r++) {
			// Nodal mass
			nodes[I[r]].mass += MP[p].N[r] * MP[p].mass;
			// Nodal momentum
			nodes[I[r]].q += MP[p].N[r] * MP[p].mass * MP[p].vel;
			// Blocked DOFs
			if (nodes[I[r]].xfixed) nodes[I[r]].q.x = 0.0;
			if (nodes[I[r]].yfixed) nodes[I[r]].q.y = 0.0;
		}
	}

	// 4) Update strain and stress
	for (size_t p = 0 ; p < MP.size() ; p++) {
		MP[p].constitutiveModel->updateStrainAndStress(MPM, p);
	}

	// 5) Compute internal and external forces
	for (size_t p = 0 ; p < MP.size() ; p++) {
		I = &(Elem[MP[p].e].I[0]);

		for (int r = 0 ; r < 8 ; r++) {
			// Internal forces
			nodes[I[r]].f += -MP[p].vol * (MP[p].stress * MP[p].gradN[r]);
			// External forces (gravity)
			nodes[I[r]].f += MP[p].mass * gravity * MP[p].N[r];

		}
	}

	//5 extra) computing velocities in nodes
	for (size_t p = 0 ; p < MP.size() ; p++) {
		I = &(Elem[MP[p].e].I[0]);
		for (int r = 0 ; r < 8 ; r++) {
			nodes[I[r]].vel = nodes[I[r]].q/nodes[I[r]].mass;
		}
	}

	// 5ter) More external forces
	MPM.boundaryConditions();
	MPM.blenderBoundaryConditions();


	for (size_t p = 0 ; p < MP.size() ; p++) {
		//sending BC forces to the nodes
		I = &(Elem[MP[p].e].I[0]);
		for (int r = 0 ; r < 8 ; r++) {
			nodes[I[r]].fb += MP[p].f * MP[p].N[r];
		}
	}

	// 6) Compute rate of momentum and update nodes
	for (size_t n = 0 ; n < liveNodeNum.size() ; n++) {

		nodes[liveNodeNum[n]].qdot = nodes[liveNodeNum[n]].fb + nodes[liveNodeNum[n]].f; //think of q as mass*vel and qdot as mass*acceleration

		if ( !(nodes[liveNodeNum[n]].xfixed) ) nodes[liveNodeNum[n]].q.x += nodes[liveNodeNum[n]].qdot.x * dt;
		else nodes[liveNodeNum[n]].qdot.x = 0.0;
		if ( !(nodes[liveNodeNum[n]].yfixed) ) nodes[liveNodeNum[n]].q.y += nodes[liveNodeNum[n]].qdot.y * dt;
		else nodes[liveNodeNum[n]].qdot.y = 0.0;
		if ( !(nodes[liveNodeNum[n]].zfixed) ) nodes[liveNodeNum[n]].q.z += nodes[liveNodeNum[n]].qdot.z * dt;
		else nodes[liveNodeNum[n]].qdot.z = 0.0;
	}

	// 7) Update material points (position and velocities)
	for (size_t p = 0 ; p < MP.size() ; p++) {
		I = &(Elem[MP[p].e].I[0]);

        //total displacement
        MP[p].total_displacement.x += MP[p].pos.x - MP[p].prev_pos.x;  //this part is giving wrong results in the vtk!
		MP[p].total_displacement.y += MP[p].pos.y - MP[p].prev_pos.y;
        MP[p].total_displacement.z += MP[p].pos.z - MP[p].prev_pos.z;

        // instant displacement
        MP[p].instant_displacement.x = MP[p].pos.x - MP[p].prev_pos.x;
		MP[p].instant_displacement.y = MP[p].pos.y - MP[p].prev_pos.y;
        MP[p].instant_displacement.z = MP[p].pos.z - MP[p].prev_pos.z;

		MP[p].prev_pos = MP[p].pos;

		double invmass;
		for (int r = 0 ; r < 8 ; r++) {
			if (nodes[I[r]].mass > tolmass) {
				invmass = 1.0 / nodes[I[r]].mass;
				//cout<<MP[p].pos<<endl;
				MP[p].pos += MP[p].N[r]*nodes[I[r]].q*invmass*dt;
				MP[p].vel += MP[p].N[r]*nodes[I[r]].qdot*invmass*dt;
			}
		}
	}

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

	// 8) Update time
	// t += dt;

}
