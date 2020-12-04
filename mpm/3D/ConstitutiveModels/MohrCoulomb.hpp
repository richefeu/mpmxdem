#ifndef MOHRCOULOMB_HPP_4F7FFG1E
#define MOHRCOULOMB_HPP_4F7FFG1E

#include "ConstitutiveModel.hpp"

struct MohrCoulomb : public ConstitutiveModel
{
private:
	double sinFrictionAngle;
	double cosFrictionAngle;
	double sinDilatancyAngle;
	double cosDilatancyAngle;

public:
	double Young;
	double Poisson;
	double FrictionAngle;
	double Cohesion;
	double DilatancyAngle;

	MohrCoulomb(double young = 200.0e6, double poisson = 0.2, double frictionAngle = 0.5, double cohesion = 0.0, double dilatancyAngle = 0.3);

	void read(std::istream & is);
	double getYoung();
	void initializeMaterialPoint(MaterialPoint & MP);
	void updateStrainAndStress(MPMbox & MPM, size_t p);
};

#endif /* end of include guard: MOHRCOULOMB_HPP_4F7FFG1E */
