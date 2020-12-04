#ifndef HOOKEELASTICITY_HPP_6CF02188
#define HOOKEELASTICITY_HPP_6CF02188

 
#include "ConstitutiveModel.hpp" 
struct HookeElasticity : public ConstitutiveModel
{
	double Young;
	double Poisson;
	
	HookeElasticity(double young = 200.0e6, double poisson = 0.2);
	
	void read(std::istream & is);
	void initializeMaterialPoint(MaterialPoint & MP);
	void updateStrainAndStress(MPMbox & MPM, size_t p);
};

#endif /* end of include guard: HOOKEELASTICITY_HPP_6CF02188 */
