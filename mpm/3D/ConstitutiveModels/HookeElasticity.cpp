 // ==== HookeElasticity ===============================================================================================
#include "HookeElasticity.hpp"

#include <../../common/factory.hpp>
static Registrar<ConstitutiveModel, HookeElasticity> registrar("HookeElasticity");

HookeElasticity::HookeElasticity(double young, double poisson) : Young(young),Poisson(poisson) { }

void HookeElasticity::read(std::istream & is)
{
	is >> Young >> Poisson;
}

void HookeElasticity::initializeMaterialPoint(MaterialPoint&) { } 

void HookeElasticity::updateStrainAndStress(MPMbox & MPM, size_t p)
{
	int * I = &(MPM.Elem[MPM.MP[p].e].I[0]);

	vec3r vn;
	mat9r dstrain;
	for (int r = 0 ; r < 8 ; r++) {
		if (MPM.nodes[I[r]].mass > MPM.tolmass) {
			vn = MPM.nodes[I[r]].q / MPM.nodes[I[r]].mass;
		} 
		else continue;
		dstrain.xx += (vn.x * MPM.MP[p].gradN[r].x) * MPM.dt;
		dstrain.xy += 0.5 * (vn.x * MPM.MP[p].gradN[r].y + vn.y * MPM.MP[p].gradN[r].x) * MPM.dt; 
		dstrain.xz += 0.5 * (vn.x * MPM.MP[p].gradN[r].z + vn.z * MPM.MP[p].gradN[r].x) * MPM.dt; 
		dstrain.yy += (vn.y * MPM.MP[p].gradN[r].y) * MPM.dt;
		dstrain.yz += 0.5 * (vn.y * MPM.MP[p].gradN[r].z + vn.z * MPM.MP[p].gradN[r].y) * MPM.dt;
		dstrain.zz += (vn.z * MPM.MP[p].gradN[r].z) * MPM.dt;
	}
	dstrain.yx = dstrain.xy;
	dstrain.zx = dstrain.xz;
	dstrain.zy = dstrain.yz;

	// Update strain
	MPM.MP[p].strain += dstrain;
	
	// Update Stress
	double a = 1.0 - Poisson;
	double b = 1.0 - 2.0 * Poisson;
	double f = Young / ((1.0 + Poisson) * b);
	MPM.MP[p].stress.xx = f * (a * MPM.MP[p].strain.xx + Poisson * MPM.MP[p].strain.yy + Poisson * MPM.MP[p].strain.zz);
	MPM.MP[p].stress.yy = f * (a * MPM.MP[p].strain.yy + Poisson * MPM.MP[p].strain.xx + Poisson * MPM.MP[p].strain.zz);
	MPM.MP[p].stress.zz = f * (a * MPM.MP[p].strain.zz + Poisson * MPM.MP[p].strain.xx + Poisson * MPM.MP[p].strain.yy);
	MPM.MP[p].stress.xy = MPM.MP[p].stress.yx = f * b/2 * MPM.MP[p].strain.xy;	
	MPM.MP[p].stress.yz = MPM.MP[p].stress.zy = f * b/2 * MPM.MP[p].strain.yz;
	MPM.MP[p].stress.zx = MPM.MP[p].stress.xz = f * b/2 * MPM.MP[p].strain.zx;	
	
}

