#ifndef MATSUOKA_HPP_4F7FFG1E
#define MATSUOKA_HPP_4F7FFG1E

#include "ConstitutiveModel.hpp"

struct Matsuoka : public ConstitutiveModel {
private:
	double sinFrictionAngle;
	double cosFrictionAngle;
	double sinDilatancyAngle;

public:
	double Young;
	double Poisson;
	double FrictionAngle;
	double Cohesion;
	double DilatancyAngle;

	Matsuoka(double young = 200.0e6, double poisson = 0.2, double frictionAngle = 0.5, double cohesion = 0.0, double dilatancyAngle = 0.3);

	void read(std::istream & is);
	void initializeMaterialPoint(MaterialPoint & MP);
	void updateStrainAndStress(MPMbox & MPM, size_t p);
};

#endif /* end of include guard: MOHRCOULOMB5_HPP_4F7FFG1E */

