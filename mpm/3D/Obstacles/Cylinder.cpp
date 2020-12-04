#include "Cylinder.hpp"
#include <Core/MPMbox.hpp>

#include <../../common/factory.hpp>
static Registrar<Obstacle, Cylinder> registrar("Cylinder");

using namespace std;
	//std::cout<<"a Circle has been constructed"<<std::endl;

void Cylinder::read(std::istream & is)
{
	is >> group >> pos >> R >> length >> direction;

	//converting direction into a unit vector
	double normDir = norm(direction);
	unitDirection.x = direction.x/normDir;
	unitDirection.y = direction.y/normDir;
	unitDirection.z = direction.z/normDir;


	/*
	std::string driveMode;
	is >> driveMode;
	if (driveMode == "freeze") {
		isFree = false;
		vel.reset();
	}
	else if (driveMode == "velocity") {
		isFree = false;
		double density;
		is >> density >> vel;
		// is >> vel;
	}
	else if (driveMode == "free") {
		isFree = true;
		double density;
		is >> density;
		mass = (4.0/3.0)*Mth::pi * R * R * R * density;
		I = 2 * mass * R * R /5;
		is >> vel >> vrot;
	}
	else { std::cerr << "@Sphere::read, driveMode " << driveMode << " is unknown!" << std::endl; }
	*/
}

void Cylinder::deleteIfInside(MPMbox &MPM) {
	int p = 0;
	bool del = false;
	for (auto i = MPM.MP.begin();i!=MPM.MP.end();) {
		vec3r c = MPM.MP[p].pos - pos;
		//first test if the point is perpendicular to the cylinder
		if (c*unitDirection > 0 or c*unitDirection < length) {

			double len = std::pow(MPM.MP[0].vol, 1/3.)/2;//  sqrt(MP.vol)/2.0;//because vol is size^3

			//finding angle between c and unitDirection
			double angleTheta = acos((c*unitDirection)/(norm(c)*norm(unitDirection)));

			//distance perpendicular to the cylinder's axis
			double dist = norm(c)*sin(angleTheta);

		    double dn = dist - R - len;
			// if (dn < 0.5)
			// std::cout<<dn<<std::endl;
			if (dn < 0.0) {
				i = MPM.MP.erase(i);
				del = true;
			}
			else {
				++i;
				++p;
			}
		}
	}
	if (del == true) std::cout<<"@Cylinder::deleteIfInside, deleted some MPs"<<std::endl;
}

bool Cylinder::touch(MaterialPoint & MP, int /*r*/, double &dn)
{
	vec3r c = MP.pos - pos;
	//first test if the point is perpendicular to the cylinder
	if (c*unitDirection < 0 or c*unitDirection > length) {
		return false;
	}

	double len = std::pow(MP.vol, 1/3.)/2;//  sqrt(MP.vol)/2.0;//because vol is size^3

	//finding angle between c and unitDirection
	double angleTheta = acos((c*unitDirection)/(norm(c)*norm(unitDirection)));

	//distance perpendicular to the cylinder's axis
	double dist = norm(c)*sin(angleTheta);

    dn = dist - R - len;
	// if (dn < 0.2) std::cout<<dn<<std::endl;
	if (dn < 0.0) {
		return true;
	}
	else return false;
}
void Cylinder::getContactFrame(MaterialPoint & MP, vec3r & N, vec3r & /*T1*/, vec3r & /*T2*/){
	// TODO: Calculate tang vectors

	vec3r c = MP.pos - pos;
	//c projected on the cylinder axis
	vec3r projAxis = pos + (unitDirection*c)* unitDirection;

	N = MP.pos - projAxis;
	N.normalize();
	//vector from axis (same 'dist' as MP ) to MP
	// vec3r temp = MP.pos - (pos + unitDirection*())

	// N = MP.pos - pos;
	// N.normalize();
	/*
	T1.x = -N.y;
	T1.y = N.x;
	T1.z = 0.0;
	T2.x = 0.0;
	T2.y = N.z;
	T2.z = -N.y;
	*/
}

void Cylinder::getContactData(MaterialPoint & /*MP*/, int , double & /*vn*/, double & /*vt1*/, double & /*vt2*/, vec3r & /*N*/, vec3r & /*T1*/, vec3r & /*T2*/)
{

}

void Cylinder::checkProximity(MPMbox & MPM)
{
	//TODO: Update this proximity in the other obstacles (use like in 2D)
	// Temporarily store the forces
	std::vector<Neighbor> Store = Neighbors;

	// Rebuild the list
	Neighbors.clear();
	Neighbor N;
	//vec3r c;
	for (size_t p = 0 ; p < MPM.MP.size() ; p++) {
		//double sumSecurDistMin = MPM.MP[p].size;
		//double sumSecurDist = MPM.MP[p].securDist + securDist;
		//if (sumSecurDist < sumSecurDistMin) sumSecurDist = sumSecurDistMin;
		//c = MPM.MP[p].pos - pos;
		//double dst = norm(c) - R;
		//std::cout<<dst<<" <? "<<sumSecurDist<<std::endl;
		//if (dst < sumSecurDist) {
			N.PointNumber = p;
			Neighbors.push_back(N);
		//}
	}

	// Get the known forces back
	size_t istore = 0;
	for (size_t inew = 0 ; inew < Neighbors.size() ; inew++) {
		while (istore < Store.size() && Neighbors[inew].PointNumber < Store[istore].PointNumber) ++istore;
		if (istore == Store.size()) break;

		if (Store[istore].PointNumber == Neighbors[inew].PointNumber) {
			//TODO: after updating the neighbor class correct this part like in 2D (check other obstacles as well)
			Neighbors[inew].fn[0] = Store[istore].fn[0];
			//Neighbors[inew].fn[1] = Store[istore].fn[1];
			//Neighbors[inew].fn[2] = Store[istore].fn[2];
			//Neighbors[inew].fn[3] = Store[istore].fn[3];

			Neighbors[inew].ft[0] = Store[istore].ft[0];
			//Neighbors[inew].ft[1] = Store[istore].ft[1];
			//Neighbors[inew].ft[2] = Store[istore].ft[2];
			//Neighbors[inew].ft[3] = Store[istore].ft[3];

			++istore;
		}
	}
}


int Cylinder::addVtkPoints(std::vector<vec3r> & coords, std::vector<int> & nbNodesOfObstacle)
{
	const int nbSectors = 75;
	double inc = Mth::_2pi / (double) nbSectors;
	vec3r P;
	//find angle difference with respect to vector 0 1 0
	// finding angles with respect to cartesian axes
	vec3r thetaUnit;
	thetaUnit.x = acos(unitDirection.x/norm(unitDirection));
	thetaUnit.y = acos(unitDirection.y/norm(unitDirection));
	thetaUnit.z = acos(unitDirection.z/norm(unitDirection));

	vec3r anglesYvector;  //this is obviously not necessary , just for clarity
	anglesYvector.x = acos(0/1);
	anglesYvector.y = acos(1/1);
	anglesYvector.z = acos(0/1);

	vec3r theta = anglesYvector - thetaUnit;

	//creating individual rotation matrices
	mat9r rotX(1, 0, 0,
			   0, cos(theta.x), -sin(theta.x),
			   0, sin(theta.x), cos(theta.x));

    mat9r rotY(cos(theta.y), 0, sin(theta.y),
	           0, 1, 0,
			   -sin(theta.y), 0, cos(theta.y));

	mat9r rotZ(cos(theta.z), -sin(theta.z), 0,
	 		   sin(theta.z), cos(theta.z), 0,
			   0, 0, 1);


	//We rotate with respect to "pos"
	mat9r rotationMat = rotZ * rotY * rotX;

	//You can construct it along the y axis
	//and then rotate it
	//FIXME Problem when direction is for example 0 1 1
	for (int i = 0 ; i < nbSectors ; i++) {
		P.x = pos.x + R * cos(i * inc);
		P.y = pos.y;
		P.z = pos.z + R * sin(i * inc);
		//rotating
		P.x = pos.x + (P.x-pos.x)*rotationMat.xx + (P.y-pos.y)*rotationMat.xy + (P.z-pos.z)*rotationMat.xz;
		P.y = pos.y + (P.x-pos.x)*rotationMat.yx + (P.y-pos.y)*rotationMat.yy + (P.z-pos.z)*rotationMat.yz;
		P.z = pos.z + (P.x-pos.x)*rotationMat.zx + (P.y-pos.y)*rotationMat.zy + (P.z-pos.z)*rotationMat.zz;
		coords.push_back(P);
		P.x = pos.x + R*cos((i+1) * inc);
		P.y = pos.y;
		P.z = pos.z + R*sin((i+1) * inc);
		//rotating
		P.x = pos.x + (P.x-pos.x)*rotationMat.xx + (P.y-pos.y)*rotationMat.xy + (P.z-pos.z)*rotationMat.xz;
		P.y = pos.y + (P.x-pos.x)*rotationMat.yx + (P.y-pos.y)*rotationMat.yy + (P.z-pos.z)*rotationMat.yz;
		P.z = pos.z + (P.x-pos.x)*rotationMat.zx + (P.y-pos.y)*rotationMat.zy + (P.z-pos.z)*rotationMat.zz;
		coords.push_back(P);
		P.x = pos.x + R*cos((i+1) * inc);
		P.y = pos.y + length;
		P.z = pos.z + R*sin((i+1) * inc);
		//rotating
		P.x = pos.x + (P.x-pos.x)*rotationMat.xx + (P.y-pos.y)*rotationMat.xy + (P.z-pos.z)*rotationMat.xz;
		P.y = pos.y + (P.x-pos.x)*rotationMat.yx + (P.y-pos.y)*rotationMat.yy + (P.z-pos.z)*rotationMat.yz;
		P.z = pos.z + (P.x-pos.x)*rotationMat.zx + (P.y-pos.y)*rotationMat.zy + (P.z-pos.z)*rotationMat.zz;
		coords.push_back(P);
		P.x = pos.x + R*cos(i * inc);
		P.y = pos.y + length;
		P.z = pos.z + R*sin(i * inc);
		//rotating
		P.x = pos.x + (P.x-pos.x)*rotationMat.xx + (P.y-pos.y)*rotationMat.xy + (P.z-pos.z)*rotationMat.xz;
		P.y = pos.y + (P.x-pos.x)*rotationMat.yx + (P.y-pos.y)*rotationMat.yy + (P.z-pos.z)*rotationMat.yz;
		P.z = pos.z + (P.x-pos.x)*rotationMat.zx + (P.y-pos.y)*rotationMat.zy + (P.z-pos.z)*rotationMat.zz;
		coords.push_back(P);

		nbNodesOfObstacle.push_back(4);
	}
	return 0;
}
