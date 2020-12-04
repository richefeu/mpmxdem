#include "VanEekelen.hpp"

#include <../../common/factory.hpp>
static Registrar<ConstitutiveModel, VanEekelen> registrar("VanEekelen");
using namespace std;

// == Mohr Coulomb 4  =================================================================================================

VanEekelen::VanEekelen(double young, double poisson, double frictionAngle, double cohesion, double dilatancyAngle, double nval)
	: Young(young),Poisson(poisson),
FrictionAngle(frictionAngle),Cohesion(cohesion),DilatancyAngle(dilatancyAngle), n(nval) { }

void VanEekelen::read(std::istream & is)
{
	is >> Young >> Poisson >> FrictionAngle >> Cohesion >> DilatancyAngle>>n;
	sinFrictionAngle  = sin(FrictionAngle);
	sinDilatancyAngle = sin(DilatancyAngle);
	cosFrictionAngle  = cos(FrictionAngle);
	cosDilatancyAngle = cos(DilatancyAngle);
	tanFrictionAngle  = tan(FrictionAngle);
}	

void VanEekelen::initializeMaterialPoint(MaterialPoint &) { }

void VanEekelen::updateStrainAndStress(MPMbox & MPM, size_t p)
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
	
	//if(p==631 && MPM.t >=0.0014)  cout<<"yieldF pre-check: "<<yieldF<<endl;
	
	// Update Stress
	double a = 1.0 - Poisson;
	double b = 1.0 - 2.0 * Poisson;
	double f = Young / ((1.0 + Poisson) * b);
	MPM.MP[p].stress.xx += f * (a * dstrain.xx + Poisson * dstrain.yy + Poisson * dstrain.zz);
	MPM.MP[p].stress.yy += f * (a * dstrain.yy + Poisson * dstrain.xx + Poisson * dstrain.zz);
	MPM.MP[p].stress.zz += f * (a * dstrain.zz + Poisson * dstrain.xx + Poisson * dstrain.yy);
	MPM.MP[p].stress.xy +=  f * b/2 * dstrain.xy;	
	MPM.MP[p].stress.yz +=  f * b/2 * dstrain.yz;
	MPM.MP[p].stress.zx +=  f * b/2 * dstrain.zx;	
	MPM.MP[p].stress.yx = MPM.MP[p].stress.xy;
	MPM.MP[p].stress.zy = MPM.MP[p].stress.yz;
	MPM.MP[p].stress.xz = MPM.MP[p].stress.zx;
	
	
	//creating a vector of vectors to find invariants in an easier way
	std::vector<std::vector<double>> stress;
	
	std::vector<double> stressline;
	stressline.push_back(MPM.MP[p].stress.xx);
	stressline.push_back(MPM.MP[p].stress.xy);
	stressline.push_back(MPM.MP[p].stress.xz);	
	stress.push_back(stressline);
	stressline.clear();
	
	stressline.push_back(MPM.MP[p].stress.yx);
	stressline.push_back(MPM.MP[p].stress.yy);
	stressline.push_back(MPM.MP[p].stress.yz);
	stress.push_back(stressline);
	stressline.clear();
	
	stressline.push_back(MPM.MP[p].stress.zx);
	stressline.push_back(MPM.MP[p].stress.zy);
	stressline.push_back(MPM.MP[p].stress.zz);
	stress.push_back(stressline);
	
	double invar1 = 0;
	double invar2 = 0;
	double invar3 = 0;
	double beta = 0;
	
	double delvar;
	//first invariant
	for (int i = 0; i<3;i++) {
		for (int j = 0; j < 3; j++)		
		{
			if (i == j) invar1 += stress[i][j];
		}
	}
	
	vector<vector<double> > devStress(3, vector<double>(3,3));
	//deviatoric tensor
	for (int i = 0; i<3;i++) {
		for (int j = 0; j < 3; j++)		
		{
			int kronecker = 0;
			if(i==j) kronecker = 1;
			devStress[i][j] = stress[i][j] - invar1/3*kronecker;					
		}
	}
	
	//second invariant
	for (int i = 0; i<3;i++) {
		for (int j = 0; j < 3; j++)		
		{
			invar2 += sqrt(0.5*devStress[i][j]*devStress[i][j]);			
		}
	}
	
	//third invariant
	for (int i = 0; i < 3;i++) {
		for (int j = 0; j < 3; j++)	{
			for (int k = 0; k < 3; k++) {
				invar3 += 1.0/3.0*devStress[i][j]*devStress[j][k]*devStress[k][i];				
			}
		}
	}
	
	//Beta
	beta = -(1.0/3.0)*asin((3*sqrt(3)/2)*(invar3/pow(invar2,3)));
	
	
	double rc = 1.0/sqrt(3.0)*(2*sinFrictionAngle/(3-sinFrictionAngle));
	double re = 1.0/sqrt(3.0)*(2*sinFrictionAngle/(3+sinFrictionAngle));
	double bval = (pow(rc/re, 1.0/n)-1)/(pow(rc/re,1.0/n)+1);
	double aval = rc/pow(1+bval,n);
	
	double m = aval*pow((1+bval*sin(3*beta)),n);
	double yieldF = invar2 + m*(invar1 - 3*Cohesion/tanFrictionAngle);
	
	//if (MPM.t>=0.001) {	
	//	
	//	cout<<yieldF<<endl;
	//	cin>>delvar;
	//}
	
	
	//df/dSij = df/dI*dI/dSij + df/dII*dII/dSij + df/dsin3B*dsin3B/dSij
	
	//Plasticity!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	if (yieldF >= 0) {
		int j = 0;
		double tolerance = 0.0001;
		double prevYieldF = yieldF;
		double criterion = 2*tolerance;
		while (criterion >= tolerance and yieldF>tolerance) {				
			j +=1;				
			//if(p==701 && MPM.t >=0.025) {
			//	cout<<"yieldF: "<<yieldF<<endl;
			//	cout<<"stress:\n"<<MPM.MP[p].stress<<endl;
			//}
			//cout<<"yieldF: "<<yieldF<<endl;
			
			//if(p==552 && MPM.t >= 0.16632) {
			//	cout<<"\nNEW ITERATION!"<<endl;
			//	cout<<"parts of yieldF:\n "<<"3c/tan "<<3*Cohesion/tanFrictionAngle<<endl;
			//	cout<<"dev stress: "<<endl;
			//	for (int i = 0; i<3;i++) {
			//		for (int j = 0; j < 3; j++)		
			//		{						
			//			cout<<devStress[i][j]<<endl;// = stress[i][j] - invar1/3*kronecker;					
			//		}
			//	}
			//}
			
			
			//calculating gradf
			vector<double> gradf;	
			//cout<<"gradf"<<endl;
			for (int i = 0; i<3;i++) {		
				for (int j = 0; j < 3; j++)	{
					double sum = 0;
					for (int k = 0; k<3;k++) {
						sum += devStress[i][k]*devStress[k][j];
						//cout<<"i: "<<i<<" j: "<<j<<" k: "<<k<<endl;
						//cout<<"calc: "<<devStress[i][k]*devStress[k][j]<<endl;
						//cout<<"sum: "<<sum<<endl;
						//cin>>delvar;
					}
						if ((i == 1 and j == 0) or (i == 2 and j == 0) or (i == 2 and j == 1)) {}			
						else {
							//cout<<"i: "<<i<<" j: "<<j<<endl;
							int kronecker = 0;
							if(i==j) kronecker = 1;
							//cout<<"final sum: "<<sum<<endl;
							//df/dsin3B
							double leftVal = aval*bval*n*pow(1+bval*sin(3*beta), n - 1.0)*(invar1 - 3*Cohesion/tanFrictionAngle);					
							//dsin3B/dSij
							double rightVal = -3*sqrt(3)/(2*pow(invar2,3))*(sum-2.0/3.0*pow(invar2,2)*kronecker-3*invar3/invar2*devStress[i][j]/(2*invar2));
							
							double value = m*kronecker + 1*devStress[i][j]/(2*invar2) + leftVal*rightVal;
							//if(p==552 && MPM.t >= 0.16632) cout<<"1: "<<m<<" 2: "<<kronecker<<" 3: "<<1 <<"\n4: "<<devStress[i][j]/(2*invar2)<<" 5: "<<leftVal<<" 6: "<<rightVal<<" sum: "<<sum<<endl;//"\ntest2: "<<pow(invar1,2)<<endl;
							//cin>>delvar;
							
							gradf.push_back(value);
						}
												
				}		
				
			}
			//sorting the gradf vector		
			swap(gradf[1], gradf[3]);
			swap(gradf[2], gradf[5]);
			
			//cout<<"stress: "<<MPM.MP[p].stress<<endl;
			
			//cout<<"gradf[0]: "<<gradf[0]<<endl;
			//cout<<"gradf[1]: "<<gradf[1]<<endl;
			//cout<<"gradf[2]: "<<gradf[2]<<endl;
			//cout<<"gradf[3]: "<<gradf[3]<<endl;
			//cout<<"gradf[4]: "<<gradf[4]<<endl;
			//cout<<"gradf[5]: "<<gradf[5]<<endl;
			//cin>>delvar;
			
			//the number 2 indicates those are related to m'
			double rc2 = 1.0/sqrt(3.0)*(2*sinDilatancyAngle/(3-sinDilatancyAngle));
			double re2 = 1.0/sqrt(3.0)*(2*sinDilatancyAngle/(3+sinDilatancyAngle));
			double bval2 = (pow(rc2/re2, 1.0/n)-1)/(pow(rc2/re2,1.0/n)+1);
			double aval2 = rc2/pow(1+bval2,n);
			double m2 = aval2*pow((1+bval2*sin(3*beta)),n);
			
			
			//calculating gradg
			vector<double> gradg;	
			//cout<<"gradg"<<endl;
			for (int i = 0; i<3;i++) {		
				for (int j = 0; j < 3; j++)	{
					double sum2 = 0;
					for (int k = 0; k<3;k++) {
						sum2 += devStress[i][k]*devStress[k][j];  //im doing this twice but leave it here to make it clearer
					}
						if ((i == 1 and j == 0) or (i == 2 and j == 0) or (i == 2 and j == 1)) {}			
						else {
							//cout<<"i: "<<i<<" j: "<<j<<endl;
							int kronecker = 0;
							if(i==j) kronecker = 1;
							
							//df/dsin3B
							double leftVal = aval2*bval2*n*pow(1+bval2*sin(3*beta), n - 1.0)*(invar1 - 3*Cohesion/tanFrictionAngle);					
							//dsin3B/dSij
							double rightVal = -3*sqrt(3)/(2*pow(invar2,3))*(sum2-2.0/3.0*pow(invar2,2)*kronecker-3*invar3/invar2*devStress[i][j]/(2*invar2));
							
							double value = m2*kronecker + 1*devStress[i][j]/(2*invar2) + leftVal*rightVal;
							
							gradg.push_back(value); 
						}
												
				}		
				
			}
			//sorting the gradf vector		
			swap(gradg[1], gradg[3]);
			swap(gradg[2], gradg[5]);
			
			
			double top_lambda = yieldF;
			double bottom_lambda = (Young*(4*gradf[0]*gradg[0] - 2*gradf[0]*gradg[1] - 2*gradf[1]*gradg[0] - 2*gradf[0]*gradg[2] + 	4*gradf[1]*gradg[1] 
			- 2*gradg[0]*gradf[2] - 2*gradf[1]*gradg[2] - 2*gradf[2]*gradg[1] + 4*gradf[2]*gradg[2] + 3*gradf[3]*gradg[3] + 3*gradf[4]*gradg[4] 
			+ 3*gradf[5]*gradg[5]))/(6*(Poisson + 1)) - (Young*(gradf[0] + gradf[1] + gradf[2])*(gradg[0] + gradg[1] + gradg[2]))/(3*(2*Poisson - 1));
	
			double lambda = top_lambda/bottom_lambda;
			//cout<<"lambda: "<<lambda<<endl;
			
			////erase
			////cout<<"gradg:\n";
			//for (size_t i = 0; i<gradg.size();i++) {
			//	//cout<<gradg[i];
			//}
			////cout<<endl;
			
			vector<double> deltaPlasticStrain;
			
			for (size_t i = 0; i<gradg.size();i++) {
				deltaPlasticStrain.push_back(lambda*gradg[i]);
				//cout<<"deltaPlasticStrain: "<<lambda*gradg[i]<<endl;
			}
			//cout<<"stress: "<<MPM.MP[p].stress<<endl;
			
						
			mat9r delta_sigma_corrector;		
			//0 = xx, 1 = yy, 2 = zz, 3 = xy, 4 = yz, 5 = zx 
			
			delta_sigma_corrector.xx = f * (a * deltaPlasticStrain[0] + Poisson * deltaPlasticStrain[1] + Poisson * deltaPlasticStrain[2]);
			delta_sigma_corrector.yy = f * (a * deltaPlasticStrain[1] + Poisson * deltaPlasticStrain[0] + Poisson * deltaPlasticStrain[2]);
			delta_sigma_corrector.zz = f * (a * deltaPlasticStrain[2] + Poisson * deltaPlasticStrain[0] + Poisson * deltaPlasticStrain[1]);
			delta_sigma_corrector.xy = delta_sigma_corrector.yx = f * b/2 * deltaPlasticStrain[3];	
			delta_sigma_corrector.yz = delta_sigma_corrector.zy = f * b/2 * deltaPlasticStrain[4];
			delta_sigma_corrector.zx = delta_sigma_corrector.xz = f * b/2 * deltaPlasticStrain[5];
						
			
			//cout<<"delta_sigma_corrector:\n"<<delta_sigma_corrector<<endl;
			
			//if(p==552 && MPM.t >= 0.16632) {
			//	cout<<"\ntime: "<<MPM.t<<"\tp: "<<p<<"\tj: "<<j<<endl;
			//	cout<<"stress: "<<MPM.MP[p].stress<<endl;
			//	//cout<<"deviatoric: "<<
			//	cout<<"invar1: "<<invar1<<" invar2: "<<invar2<<" invar3: "<<invar3<<endl;
			//	cout<<"m: "<<m<<" rc: "<<rc<<" re: "<<re<<" aval: "<<aval<<" bval: "<<bval<<endl;
			//	cout<<"lambda: "<<lambda<<endl;
			//	cout<<"gradf: "<<endl;
			//	for (size_t i = 0; i<gradg.size();i++) {
			//		cout<<gradf[i]<<endl;
			//	}
			//	
			//}
			
			MPM.MP[p].stress -= delta_sigma_corrector;
			
			//creating a vector of vectors to find invariants in an easier way
			stress.clear();
			std::vector<std::vector<double>> stress;
			
			std::vector<double> stressline;
			stressline.push_back(MPM.MP[p].stress.xx);
			stressline.push_back(MPM.MP[p].stress.xy);
			stressline.push_back(MPM.MP[p].stress.xz);	
			stress.push_back(stressline);
			stressline.clear();
			
			stressline.push_back(MPM.MP[p].stress.yx);
			stressline.push_back(MPM.MP[p].stress.yy);
			stressline.push_back(MPM.MP[p].stress.yz);
			stress.push_back(stressline);
			stressline.clear();
			
			stressline.push_back(MPM.MP[p].stress.zx);
			stressline.push_back(MPM.MP[p].stress.zy);
			stressline.push_back(MPM.MP[p].stress.zz);
			stress.push_back(stressline);
		
			
			invar1 =0;
			//first invariant
			for (int i = 0; i<3;i++) {
				for (int j = 0; j < 3; j++)		
				{
					if (i == j) invar1 += stress[i][j];
				}
			}
			//cout<<"invar1: "<<invar1<<endl;
			//devStress.clear();
			//vector<vector<double> > devStress(3, vector<double>(3,3));
			
			//deviatoric tensor
			for (int i = 0; i<3;i++) {
				for (int j = 0; j < 3; j++)		
				{
					int kronecker = 0;
					if(i==j) kronecker = 1;
					devStress[i][j] = stress[i][j] - invar1/3*kronecker;					
				}
			}
			invar2 = 0;
			//second invariant
			for (int i = 0; i<3;i++) {
				for (int j = 0; j < 3; j++)		
				{
					invar2 += sqrt(0.5*devStress[i][j]*devStress[i][j]);			
				}
			}
			
			//third invariant
			invar3 = 0;
			for (int i = 0; i < 3;i++) {
				for (int j = 0; j < 3; j++)	{
					for (int k = 0; k < 3; k++) {
						invar3 += 1.0/3.0*devStress[i][j]*devStress[j][k]*devStress[k][i];				
					}
				}
			}
				
			//Beta
			//beta=0;
			beta = -(1.0/3.0)*asin((3*sqrt(3)/2)*(invar3/pow(invar2,3)));	
			
			//m
			//m = 0;
			m = aval*pow((1+bval*sin(3*beta)),n);
			
			//cout<<"invar2: "<<invar2<<endl;
			
			//if(p==286 /*&& MPM.step%50 == 0*/) MPM.logFile<<MPM.t<<" "<<yieldF<<" ";
			//if(p==552 && MPM.t >= 0.16632) cout<<"yieldF: "<<yieldF<<endl;
			
			yieldF = invar2 + m*(invar1 - 3*Cohesion/tanFrictionAngle);
			//if(p==286 /*&& MPM.step%50 == 0*/) MPM.logFile<<yieldF<<endl;
			//if(p==552 && MPM.t >= 0.16632) cout<<"corrected yieldF: "<<yieldF<<endl;
			
			
			criterion = fabs(yieldF- prevYieldF)/prevYieldF;  
			//if(p==552 && MPM.t >= 0.16632) cout<<"criterion: "<<criterion<<endl;
			
			if (j >5000 ) {std::cout<<"j = "<<j<<"\tp = "<<p<<"\nt: "<<MPM.t<<std::endl; std::cin>>delvar;}
			
			//if(p==286 && MPM.t >= 0.10268) cin>>delvar;
			
			//if(p==701 && MPM.t >=0.025) { 
			//	
			//	cout<<"corrected yieldF: "<<yieldF<<"\ntime: "<<MPM.t<<endl;			
			//	cin>>delvar;
			//}
				
		}
		
	}
	
}


