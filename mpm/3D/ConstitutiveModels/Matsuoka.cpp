#include "Matsuoka.hpp"

#include <../../common/factory.hpp>
static Registrar<ConstitutiveModel, Matsuoka> registrar("Matsuoka");
using namespace std;

// =============================================

// =============================================

Matsuoka::Matsuoka(double young, double poisson, double frictionAngle, double cohesion, double dilatancyAngle):
	Young(young), Poisson(poisson), FrictionAngle(frictionAngle), Cohesion(cohesion), DilatancyAngle(dilatancyAngle)
{
	sinFrictionAngle  = sin(FrictionAngle);
	sinDilatancyAngle = sin(DilatancyAngle);
	cosFrictionAngle  = cos(FrictionAngle);
}

void Matsuoka::read(std::istream & is)
{
	is >> Young >> Poisson >> FrictionAngle >> Cohesion >> DilatancyAngle;
	sinFrictionAngle  = sin(FrictionAngle);
	sinDilatancyAngle = sin(DilatancyAngle);
	cosFrictionAngle  = cos(FrictionAngle);
}

void Matsuoka::initializeMaterialPoint(MaterialPoint &) { }

void Matsuoka::updateStrainAndStress(MPMbox & MPM, size_t p)
{
	// Get pointer to the first of the 8 nodes
	int * I = &(MPM.Elem[MPM.MP[p].e].I[0]);

	// Compute a strain increment (during dt) from the 8 node-velocities
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

	MPM.MP[p].deltaStrain = dstrain;

	MPM.MP[p].strain += dstrain;

	MPM.MP[p].prevStress = MPM.MP[p].stress;

	//      |De11 De12 De13 0    0    0    |       |a        Poisson  Poisson  0   0   0   |  with a = 1 - Poisson
	//      |De12 De22 De23 0    0    0    |       |Poisson  a        Poisson  0   0   0   |       b = 1 - 2Poisson
	// De = |De13 De23 De33 0    0    0    | = f * |Poisson  Poisson  a        0   0   0   |
	//      |0    0    0    De44 0    0    |       |0        0        0        b/2 0   0   |  and f = Young / (1 + 2Poisson)
	//      |0    0    0    0    De55 0    |       |0        0        0        0   b/2 0   |
	//      |0    0    0    0    0    De66 |       |0        0        0        0   0   b/2 |
	double a = 1.0 - Poisson;
	double b = (1.0 - 2.0 * Poisson);
	double f = Young / ((1.0 + Poisson) * b);
	double De11 = f * a;
	double De22 = De11;
	double De33 = De11;
	double De12 = f * Poisson;
	double De13 = De12;
	double De23 = De12;
	double De44 = f * 0.5 * b;
	double De55 = De44;
	double De66 = De44;

	// Trial stress
	MPM.MP[p].stress.xx += De11 * dstrain.xx + De12 * dstrain.yy + De13 * dstrain.zz;
	MPM.MP[p].stress.yy += De12 * dstrain.xx + De22 * dstrain.yy + De23 * dstrain.zz;
	MPM.MP[p].stress.zz += De13 * dstrain.xx + De12 * dstrain.yy + De33 * dstrain.zz;
	MPM.MP[p].stress.xy += De44 * dstrain.xy;
	MPM.MP[p].stress.yz += De55 * dstrain.yz;
	MPM.MP[p].stress.zx += De66 * dstrain.zx;
	// Symmetry
	MPM.MP[p].stress.yx = MPM.MP[p].stress.xy;
	MPM.MP[p].stress.zy = MPM.MP[p].stress.yz;
	MPM.MP[p].stress.xz = MPM.MP[p].stress.zx;
	//std::cout<<"p: "<<p<<" t: "<<MPM.t<<std::endl;
	double sgn = 1.0;
	// Compute Yield function
	double decalCoh = Cohesion * cosFrictionAngle / sinFrictionAngle;
	double Third = 1.0 / 3.0;
	double Sxx = sgn * (MPM.MP[p].stress.xx - decalCoh);
	double Syy = sgn * (MPM.MP[p].stress.yy - decalCoh);
	double Szz = sgn * (MPM.MP[p].stress.zz - decalCoh);
	double Sxy = MPM.MP[p].stress.xy;
	double Sxz = MPM.MP[p].stress.xz;
	double Syz = MPM.MP[p].stress.yz;
	double press = -Third * (Sxx + Syy + Szz);
	double sigII = -(Sxx * Syy + Syy * Szz + Sxx * Szz) + Sxy * Sxy + Syz * Syz + Sxz * Sxz;
	double sigIII = (Sxx * Syy * Szz - Syy * Sxz * Sxz - Sxx * Syz * Syz - Szz * Sxy * Sxy + 2.0 * Sxy * Syz * Sxz);
	double Ap = (9.0 - sinFrictionAngle * sinFrictionAngle) / (1.0 - sinFrictionAngle * sinFrictionAngle);
	double yieldF = -3.0 * press * sigII + Ap * sigIII;



	MPM.MP[p].yieldFunctionUncorrected = yieldF;
	//std::cout<<"yield: "<<yieldF<<std::endl;
	if (yieldF > 0.0) {
		//std::cout<<"p: "<<p<<std::endl;
		double q, MP;
		double sqrt1_2 = sqrt(0.5);
		double sqrt2_3 = sqrt(2.0 / 3.0);
		double sqrt3_2 = sqrt(3.0 / 2.0);
		double diff_xx_yy, diff_xx_zz, diff_yy_zz;
		double gradfxx, gradfyy, gradfzz, gradfxy, gradfxz, gradfyz;
		double gradgxx, gradgyy, gradgzz, gradgxy, gradgxz, gradgyz;
		double lambdaDenominator, lambda;

		int iter = 0;
		while (yieldF > 1e-10) {
			iter++;
			
			if (iter > 50) break;
			
			//std::cout<<"iter "<< iter << "   p " << p << "    F = " <<yieldF<<std::endl;
			//std::cout<<"stress:\n"<< MPM.MP[p].stress.xx<<" "<<MPM.MP[p].stress.xy<<" "<<MPM.MP[p].stress.xz<<std::endl;
			//std::cout<<MPM.MP[p].stress.yx<<" "<<MPM.MP[p].stress.yy<<" "<<MPM.MP[p].stress.yz<<std::endl;
			//std::cout<<MPM.MP[p].stress.zx<<" "<<MPM.MP[p].stress.zy<<" "<<MPM.MP[p].stress.zz<<std::endl;
	
			Sxx = sgn * (MPM.MP[p].stress.xx - decalCoh);
			Syy = sgn * (MPM.MP[p].stress.yy - decalCoh);
			Szz = sgn * (MPM.MP[p].stress.zz - decalCoh);
			Sxy = MPM.MP[p].stress.xy;
			Sxz = MPM.MP[p].stress.xz;
			Syz = MPM.MP[p].stress.yz;
			diff_xx_yy = Sxx - Syy;
			diff_xx_zz = Sxx - Szz;
			diff_yy_zz = Syy - Szz;
			press = -Third * (Sxx + Syy + Szz);
			q = sqrt1_2 * sqrt(diff_xx_yy * diff_xx_yy + diff_xx_zz * diff_xx_zz + diff_yy_zz * diff_yy_zz + 6.0 * (Sxy * Sxy + Sxz * Sxz + Syz * Syz));

			sigII = -(Sxx * Syy + Syy * Szz + Sxx * Szz) + Sxy * Sxy + Syz * Syz + Sxz * Sxz;
			sigIII = (Sxx * Syy * Szz - Syy * Sxz * Sxz - Sxx * Syz * Syz - Szz * Sxy * Sxy + 2.0 * Sxy * Syz * Sxz);
			Ap = (9.0 - sinFrictionAngle * sinFrictionAngle) / (1.0 - sinFrictionAngle * sinFrictionAngle);

			gradfxx = sigII + 3.0 * press * (Syy + Szz) + Ap * (Syy * Szz - Syz * Syz);
			gradfyy = sigII + 3.0 * press * (Sxx + Szz) + Ap * (Sxx * Szz - Sxz * Sxz);
			gradfzz = sigII + 3.0 * press * (Sxx + Syy) + Ap * (Sxx * Syy - Sxy * Sxy);
			gradfxy = -6.0 * press * Sxy + Ap * (-2.0 * Szz * Sxy + 2.0 * Syz * Sxz);
			gradfxz = -6.0 * press * Sxz + Ap * (-2.0 * Syy * Sxz + 2.0 * Sxy * Syz);
			gradfyz = -6.0 * press * Syz + Ap * (-2.0 * Sxx * Syz + 2.0 * Sxy * Sxz);

			MP = 4.0 * sinDilatancyAngle / (3.0 * sqrt(6.0 + 2.0 * sinDilatancyAngle * sinDilatancyAngle));

			gradgxx = MP + sqrt3_2 * (Sxx + press) / q;
			gradgyy = MP + sqrt3_2 * (Syy + press) / q;
			gradgzz = MP + sqrt3_2 * (Szz + press) / q;
			gradgxy = 3.0 * sqrt2_3 * Sxy / q;
			gradgxz = 3.0 * sqrt2_3 * Sxz / q;
			gradgyz = 3.0 * sqrt2_3 * Syz / q;

			lambdaDenominator = gradfxx * (De11 * gradgxx + De12 * gradgyy + De13 * gradgzz)
			                  + gradfyy * (De12 * gradgxx + De22 * gradgyy + De23 * gradgzz)
			                  + gradfzz * (De13 * gradgxx + De23 * gradgyy + De33 * gradgzz)
			                  + gradfxy * De44 * gradgxy
			                  + gradfxz * De55 * gradgxz
			                  + gradfyz * De66 * gradgyz;
			lambda = yieldF / lambdaDenominator;

			mat9r deltaPlasticStrain;
			deltaPlasticStrain.xx = lambda * gradgxx;
			deltaPlasticStrain.yy = lambda * gradgyy;
			deltaPlasticStrain.zz = lambda * gradgzz;
			deltaPlasticStrain.xy = 0.5 * lambda * gradgxy;
			deltaPlasticStrain.xz = 0.5 * lambda * gradgxz;
			deltaPlasticStrain.yz = 0.5 * lambda * gradgyz;
			// Symmetry
			deltaPlasticStrain.yx = deltaPlasticStrain.xy;
			deltaPlasticStrain.zx = deltaPlasticStrain.xz;
			deltaPlasticStrain.zy = deltaPlasticStrain.yz;

			MPM.MP[p].plasticStrain += deltaPlasticStrain;

			mat9r delta_sigma_corrector;
			delta_sigma_corrector.xx = De11 * deltaPlasticStrain.xx + De12 * deltaPlasticStrain.yy + De13 * deltaPlasticStrain.zz;
			delta_sigma_corrector.yy = De12 * deltaPlasticStrain.xx + De22 * deltaPlasticStrain.yy + De23 * deltaPlasticStrain.zz;
			delta_sigma_corrector.zz = De13 * deltaPlasticStrain.xx + De12 * deltaPlasticStrain.yy + De33 * deltaPlasticStrain.zz;
			delta_sigma_corrector.xy = De44 * deltaPlasticStrain.xy;
			delta_sigma_corrector.yz = De55 * deltaPlasticStrain.yz;
			delta_sigma_corrector.zx = De66 * deltaPlasticStrain.zx;
			// Symmetry
			delta_sigma_corrector.yx = delta_sigma_corrector.xy;
			delta_sigma_corrector.zy = delta_sigma_corrector.yz;
			delta_sigma_corrector.xz = delta_sigma_corrector.zx;

			MPM.MP[p].stress -= delta_sigma_corrector;

			// New value of the yield function
			Sxx = sgn * (MPM.MP[p].stress.xx - decalCoh);
			Syy = sgn * (MPM.MP[p].stress.yy - decalCoh);
			Szz = sgn * (MPM.MP[p].stress.zz - decalCoh);
			Sxy = MPM.MP[p].stress.xy;
			Sxz = MPM.MP[p].stress.xz;
			Syz = MPM.MP[p].stress.yz;
			press = -Third * (Sxx + Syy + Szz);
			sigII = -(Sxx * Syy + Syy * Szz + Sxx * Szz) + Sxy * Sxy + Syz * Syz + Sxz * Sxz;
			sigIII = (Sxx * Syy * Szz - Syy * Sxz * Sxz - Sxx * Syz * Syz - Szz * Sxy * Sxy + 2.0 * Sxy * Syz * Sxz);
			Ap = (9.0 - sinFrictionAngle * sinFrictionAngle) / (1.0 - sinFrictionAngle * sinFrictionAngle);
			yieldF = -3.0 * press * sigII + Ap * sigIII;

		} // end iterations
		//std::cout<<"p: "<<p<<std::endl;
		MPM.MP[p].yieldFunctionCorrected = yieldF;
		// MPM.MP[p].yieldFunctionUncorrected = -1.0;

	} // end if yieldF > 0.0
}


