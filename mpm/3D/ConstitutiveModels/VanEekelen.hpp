#ifndef VanEekelen_HPP_4F7FFG1E
#define VanEekelen_HPP_4F7FFG1E

#include "ConstitutiveModel.hpp"

struct VanEekelen : public ConstitutiveModel
{
private:
	double sinFrictionAngle;
	double cosFrictionAngle;
	double sinDilatancyAngle;
	double cosDilatancyAngle;
	double tanFrictionAngle;
	
public:
	double Young;
	double Poisson;
	double FrictionAngle;
	double Cohesion;
	double DilatancyAngle;
	double n;
	
	VanEekelen(double young = 200.0e6, double poisson = 0.2, double frictionAngle = 0.5, double cohesion = 0.0, double dilatancyAngle = 0.3, double nval = -0.229);
	
	void read(std::istream & is);
	void initializeMaterialPoint(MaterialPoint & MP);
	void updateStrainAndStress(MPMbox & MPM, size_t p);
};

#endif /* end of include guard: VanEekelen_HPP_4F7FFG1E */

