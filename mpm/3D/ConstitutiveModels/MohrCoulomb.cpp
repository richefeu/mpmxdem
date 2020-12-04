#include "MohrCoulomb.hpp"

#include <../../common/factory.hpp>
static Registrar<ConstitutiveModel, MohrCoulomb> registrar("MohrCoulomb");
using namespace std;

// == Mohr Coulomb   =================================================================================================

MohrCoulomb::MohrCoulomb(double young, double poisson, double frictionAngle, double cohesion, double dilatancyAngle)
	: Young(young),Poisson(poisson),
FrictionAngle(frictionAngle),Cohesion(cohesion),DilatancyAngle(dilatancyAngle) { }

void MohrCoulomb::read(std::istream & is)
{
	is >> Young >> Poisson >> FrictionAngle >> Cohesion >> DilatancyAngle;
	sinFrictionAngle  = sin(FrictionAngle);
	sinDilatancyAngle = sin(DilatancyAngle);
	cosFrictionAngle  = cos(FrictionAngle);
	cosDilatancyAngle = cos(DilatancyAngle);
}

double MohrCoulomb::getYoung(){return Young;}

void MohrCoulomb::initializeMaterialPoint(MaterialPoint &) { }

void MohrCoulomb::updateStrainAndStress(MPMbox & MPM, size_t p)
{

	//WORKS BETTER WITH ASSOCIATED PLASTICITY
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

	//		|De11	De12	De13	0		0		0	|
	//		|De21	De22	De23	0		0		0	|
	//		|De31	De32	De33	0		0		0	|
	// De = |0		0		0		De44	0		0	|
	//		|0		0		0		0		De55	0	|
	//		|0		0		0		0		0		De66|

	//		 |a			Poisson		Poisson		0		0		0	|
	//       |Poisson	a			Poisson		0		0		0	|  with a = 1-Poisson
	//       |Poisson	Poisson		a			0		0		0	|       b = 1-2Poisson
	// = f * |0			0			0			b/2		0		0	|   and f = Young/(1+2Poisson)
	//       |0			0			0			0		b/2		0	|
	//       |0			0			0			0		0		b/2 |

	double a = 1.0 - Poisson;
	double b = (1.0 - 2.0 * Poisson);
	double f = Young / ((1.0 + Poisson) * b);
	double De11 = f * a;
	double De12 = f * Poisson;
	double De13 = De12;
	double De21 = De12;
	double De22 = De11;
	double De23 = De12;
	double De31 = De12;
	double De32 = De12;
	double De33 = De11;
	double De44 = f * 0.5 * b;
	double De55 = De44;
	double De66 = De44;


	// Update Stress

	MPM.MP[p].stress.xx += De11 * dstrain.xx + De12 * dstrain.yy + De13 * dstrain.zz;
	MPM.MP[p].stress.yy += De21 * dstrain.xx + De22 * dstrain.yy + De23 * dstrain.zz;
	MPM.MP[p].stress.zz += De31 * dstrain.xx + De32 * dstrain.yy + De33 * dstrain.zz;
	MPM.MP[p].stress.xy += De44 * dstrain.xy;
	MPM.MP[p].stress.yz += De55 * dstrain.yz;
	MPM.MP[p].stress.zx += De66 * dstrain.zx;
	MPM.MP[p].stress.yx = MPM.MP[p].stress.xy;
	MPM.MP[p].stress.zy = MPM.MP[p].stress.yz;
	MPM.MP[p].stress.xz = MPM.MP[p].stress.zx;


	mat9r eigvec;
	vec3r eigval;
	MPM.MP[p].stress.sorted_sym_eigen(eigvec, eigval);
	mat9r eigvalm;  //tensor to put the eigenval (vec3r) in matrix form
	eigvalm.xx = eigval.x;
	eigvalm.yy = eigval.y;
	eigvalm.zz = eigval.z;
	//yield Function found in thesis Ceccato
	double yieldF1 = 0.5 * fabs(eigvalm.yy-eigvalm.zz) + 0.5 * (eigvalm.yy + eigvalm.zz)*sinFrictionAngle-Cohesion*cosFrictionAngle;
	double yieldF2 = 0.5 * fabs(eigvalm.xx-eigvalm.zz) + 0.5 * (eigvalm.xx + eigvalm.zz)*sinFrictionAngle-Cohesion*cosFrictionAngle;
	double yieldF3 = 0.5 * fabs(eigvalm.xx-eigvalm.yy) + 0.5 * (eigvalm.xx + eigvalm.yy)*sinFrictionAngle-Cohesion*cosFrictionAngle;
	mat9r deltaPlasticStrain;

	//************** case 1 *****************
	if (yieldF1 > 0.0) {
		int iter = 0;
		while (iter < 50 and yieldF1 > 1e-10) { // Actually a single iteration should be okay
			iter++;

			double diff_f1 = eigvalm.yy - eigvalm.zz;
			double gradf1yy = 0.5 * diff_f1 / fabs(diff_f1) + 0.5*sinFrictionAngle;
			double gradf1zz = 0.5*sinFrictionAngle - 0.5*diff_f1/fabs(diff_f1);

			double gradg1xx = 0;
			double gradg1yy = 0.5 * diff_f1 / fabs(diff_f1) + 0.5*sinDilatancyAngle;
			double gradg1zz = 0.5*sinDilatancyAngle - 0.5*diff_f1/fabs(diff_f1);

			double bottom_lambda = gradf1yy * (De22 * gradg1yy + De23 * gradg1zz)
			                     + gradf1zz * (De32 * gradg1yy + De33 * gradg1zz);
			double lambda = yieldF1 / bottom_lambda;

			//mat4 deltaPlasticStrain;
			deltaPlasticStrain.xx = lambda * gradg1xx;
			deltaPlasticStrain.yy = lambda * gradg1yy;
			deltaPlasticStrain.zz = lambda * gradg1zz;
			deltaPlasticStrain.xy = 0;
			deltaPlasticStrain.yz = 0;
			deltaPlasticStrain.zx = 0;
			deltaPlasticStrain.yx = 0;
			deltaPlasticStrain.zy = 0;
			deltaPlasticStrain.xz = 0;

			//MPM.MP[p].plasticStrain += deltaPlasticStrain;

			//Updating total strain for later use
			//MPM.MP[p].elasticStrain = MPM.MP[p].strain - deltaPlasticStrain;

			// Correcting state of stress
			mat9r delta_sigma_corrector;

			delta_sigma_corrector.xx = De11 * deltaPlasticStrain.xx + De12 * deltaPlasticStrain.yy + De13 * deltaPlasticStrain.zz;
			delta_sigma_corrector.yy = De21 * deltaPlasticStrain.xx + De22 * deltaPlasticStrain.yy + De23 * deltaPlasticStrain.zz;
			delta_sigma_corrector.zz = De31 * deltaPlasticStrain.xx + De32 * deltaPlasticStrain.yy + De33 * deltaPlasticStrain.zz;
			delta_sigma_corrector.xy = De44 * deltaPlasticStrain.xy;
			delta_sigma_corrector.yz = De55 * deltaPlasticStrain.yz;
			delta_sigma_corrector.zx = De66 * deltaPlasticStrain.zx;
			delta_sigma_corrector.yx = delta_sigma_corrector.xy;
			delta_sigma_corrector.zy = delta_sigma_corrector.yz;
			delta_sigma_corrector.xz = delta_sigma_corrector.zx;

			//correct principal stresses
			eigvalm -= delta_sigma_corrector;
			//MPM.MP[p].stress -= delta_sigma_corrector;
			MPM.MP[p].plasticStress += delta_sigma_corrector;
			// New value of the yield function

			yieldF1 = 0.5 * fabs(eigvalm.yy-eigvalm.zz) + 0.5 * (eigvalm.yy + eigvalm.zz)*sinFrictionAngle-Cohesion*cosFrictionAngle;
			//std::cout<<"iter1: "<<iter<<"\tyield: "<<yieldF1<<std::endl;
		} // end iterations
	}

	//************** case 2 *****************
	if (yieldF2 > 0.0) {
		int iter = 0;
		while (iter < 50 and yieldF2 > 1e-10) { // Actually a single iteration should be okay
			iter++;

			double diff_f2 = eigvalm.xx - eigvalm.zz;
			double gradf2xx = 0.5 * diff_f2/fabs(diff_f2) + 0.5*sinFrictionAngle;
			double gradf2zz = 0.5*sinFrictionAngle - 0.5*diff_f2/fabs(diff_f2);

			double gradg2xx = 0.5 * diff_f2/fabs(diff_f2) + 0.5*sinDilatancyAngle;
			double gradg2yy = 0;
			double gradg2zz = 0.5*sinDilatancyAngle - 0.5*diff_f2/fabs(diff_f2);

			double bottom_lambda = gradf2xx * (De11 * gradg2xx + De13 * gradg2zz)
			                     + gradf2zz * (De31 * gradg2xx + De33 * gradg2zz);

			double lambda = yieldF2 / bottom_lambda;

			//mat4 deltaPlasticStrain;
			deltaPlasticStrain.xx = lambda * gradg2xx;
			deltaPlasticStrain.yy = lambda * gradg2yy;
			deltaPlasticStrain.zz = lambda * gradg2zz;
			deltaPlasticStrain.xy = 0;
			deltaPlasticStrain.yz = 0;
			deltaPlasticStrain.zx = 0;
			deltaPlasticStrain.yx = 0;
			deltaPlasticStrain.zy = 0;
			deltaPlasticStrain.xz = 0;

			//MPM.MP[p].plasticStrain += deltaPlasticStrain;

			//Updating total strain for later use
			//MPM.MP[p].elasticStrain = MPM.MP[p].strain - deltaPlasticStrain;

			// Correcting state of stress
			mat9r delta_sigma_corrector;

			delta_sigma_corrector.xx = De11 * deltaPlasticStrain.xx + De12 * deltaPlasticStrain.yy + De13 * deltaPlasticStrain.zz;
			delta_sigma_corrector.yy = De21 * deltaPlasticStrain.xx + De22 * deltaPlasticStrain.yy + De23 * deltaPlasticStrain.zz;
			delta_sigma_corrector.zz = De31 * deltaPlasticStrain.xx + De32 * deltaPlasticStrain.yy + De33 * deltaPlasticStrain.zz;
			delta_sigma_corrector.xy = De44 * deltaPlasticStrain.xy;
			delta_sigma_corrector.yz = De55 * deltaPlasticStrain.yz;
			delta_sigma_corrector.zx = De66 * deltaPlasticStrain.zx;
			delta_sigma_corrector.yx = delta_sigma_corrector.xy;
			delta_sigma_corrector.zy = delta_sigma_corrector.yz;
			delta_sigma_corrector.xz = delta_sigma_corrector.zx;

            //cout<<"prevStress: "<<endl<<eigvalm<<endl;
			eigvalm -= delta_sigma_corrector;
            //cout<<"newStress: "<<endl<<eigvalm<<endl;
            //cout<<"delta_sigma_corrector: "<<endl<<delta_sigma_corrector<<endl;
			MPM.MP[p].plasticStress += delta_sigma_corrector;
			// New value of the yield function

			yieldF2 = 0.5 * fabs(eigvalm.xx-eigvalm.zz) + 0.5 * (eigvalm.xx + eigvalm.zz)*sinFrictionAngle-Cohesion*cosFrictionAngle;
			//std::cout<<"iter2: "<<iter<<"\tyield: "<<yieldF2<<std::endl;
            //double delvar;
            //cin>>delvar;
		} // end iterations
	}

	//************** case 3 *****************
	if (yieldF3 > 0.0) {
		int iter = 0;
		while (iter < 50 and yieldF3 > 1e-10) { // Actually a single iteration should be okay
			iter++;

			double diff_f3 = eigvalm.xx - eigvalm.yy;
			double gradf3xx = 0.5*diff_f3/fabs(diff_f3) + 0.5*sinFrictionAngle;
			double gradf3yy = 0.5*sinFrictionAngle - 0.5*diff_f3/fabs(diff_f3);

			double gradg3xx = 0.5*diff_f3/fabs(diff_f3) + 0.5*sinDilatancyAngle;
			double gradg3yy = 0.5*sinDilatancyAngle - 0.5*diff_f3/fabs(diff_f3);
			double gradg3zz = 0;

			double bottom_lambda = gradf3xx * (De11 * gradg3xx + De12 * gradg3yy)
			                     + gradf3yy * (De21 * gradg3xx + De22 * gradg3yy);

			double lambda = yieldF3 / bottom_lambda;

			//mat4 deltaPlasticStrain;
			deltaPlasticStrain.xx = lambda * gradg3xx;
			deltaPlasticStrain.yy = lambda * gradg3yy;
			deltaPlasticStrain.zz = lambda * gradg3zz;
			deltaPlasticStrain.xy = 0;
			deltaPlasticStrain.yz = 0;
			deltaPlasticStrain.zx = 0;
			deltaPlasticStrain.yx = 0;
			deltaPlasticStrain.zy = 0;
			deltaPlasticStrain.xz = 0;

			//MPM.MP[p].plasticStrain += deltaPlasticStrain;

			//Updating total strain for later use
			//MPM.MP[p].elasticStrain = MPM.MP[p].strain - deltaPlasticStrain;

			// Correcting state of stress
			mat9r delta_sigma_corrector;

			delta_sigma_corrector.xx = De11 * deltaPlasticStrain.xx + De12 * deltaPlasticStrain.yy + De13 * deltaPlasticStrain.zz;
			delta_sigma_corrector.yy = De21 * deltaPlasticStrain.xx + De22 * deltaPlasticStrain.yy + De23 * deltaPlasticStrain.zz;
			delta_sigma_corrector.zz = De31 * deltaPlasticStrain.xx + De32 * deltaPlasticStrain.yy + De33 * deltaPlasticStrain.zz;
			delta_sigma_corrector.xy = De44 * deltaPlasticStrain.xy;
			delta_sigma_corrector.yz = De55 * deltaPlasticStrain.yz;
			delta_sigma_corrector.zx = De66 * deltaPlasticStrain.zx;
			delta_sigma_corrector.yx = delta_sigma_corrector.xy;
			delta_sigma_corrector.zy = delta_sigma_corrector.yz;
			delta_sigma_corrector.xz = delta_sigma_corrector.zx;


			eigvalm -= delta_sigma_corrector;
			MPM.MP[p].plasticStress += delta_sigma_corrector;
			// New value of the yield function

			yieldF3 = 0.5 * fabs(eigvalm.xx-eigvalm.yy) + 0.5 * (eigvalm.xx + eigvalm.yy)*sinFrictionAngle-Cohesion*cosFrictionAngle;
			//std::cout<<"iter3: "<<iter<<"\tyield: "<<yieldF3<<std::endl;;
		} // end iterations
	}


	//return to original stress space
	MPM.MP[p].stress = eigvec*eigvalm*(eigvec.get_inverse());

}
