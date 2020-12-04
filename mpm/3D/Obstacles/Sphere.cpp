#include "Sphere.hpp"
#include <Core/MPMbox.hpp>

#include <../../common/factory.hpp>
static Registrar<Obstacle, Sphere> registrar("Sphere");

using namespace std;
Sphere::Sphere() {
	//std::cout<<"a Circle has been constructed"<<std::endl;
}
void Sphere::read(std::istream & is)
{
	is >> group >> pos >> R;
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

}

bool Sphere::touch(MaterialPoint & MP, int /*r*/, double &dn)
{
  vec3r c = MP.pos - pos;
	double len = std::pow(MP.vol, 1/3.)/2;//  sqrt(MP.vol)/2.0;//because vol is size^3

  dn = norm(c) - R - len;
	if (dn < 0.0) return true;
	else return false;
}
void Sphere::getContactFrame(MaterialPoint & MP, vec3r & N, vec3r & T1, vec3r & T2){
	N = MP.pos - pos;
	N.normalize();
	T1.x = -N.y;
	T1.y = N.x;
	T1.z = 0.0;
	T2.x = 0.0;
	T2.y = N.z;
	T2.z = -N.y;


}
void Sphere::getContactData(MaterialPoint & /*MP*/, int , double & /*vn*/, double & /*vt1*/, double & /*vt2*/, vec3r & /*N*/, vec3r & /*T1*/, vec3r & /*T2*/)
{

}

void Sphere::checkProximity(MPMbox & MPM)
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


int Sphere::addVtkPoints(std::vector<vec3r> & coords, std::vector<int> & nbNodesOfObstacle)
{
	/*
	//Part used in plane
	coords.push_back(pos0);
	coords.push_back(pos1);
	coords.push_back(pos2);
	coords.push_back(pos3);
	nbNodesOfObstacle.push_back(4);
	*/

	const int nbSectors = 75;
	const int nbSectors2 = 75;
	vec3r P;
	double inc = Mth::_2pi / (double) nbSectors;
	double inc2 = Mth::pi/(double) nbSectors2;

	//coords.push_back(pos);
	for (int j = 0; j < nbSectors2; j++) {
		for (int i = 0 ; i < nbSectors ; i++) {
			P.x = pos.x + R * cos(i * inc) * cos(j*inc2);
			P.y = pos.y + R * sin(i * inc);
			P.z = pos.z + R * cos(i * inc) * sin(j*inc2);
			coords.push_back(P);
			P.x = pos.x + R * cos((i+1) * inc) * cos(j*inc2);
			P.y = pos.y + R * sin((i+1) * inc);
			P.z = pos.z + R * cos((i+1) * inc) * sin(j*inc2);
			coords.push_back(P);
			P.x = pos.x + R * cos((i+1) * inc) * cos((j+1)*inc2);
			P.y = pos.y + R * sin((i+1) * inc);
			P.z = pos.z + R * cos((i+1) * inc) * sin((j+1)*inc2);
			coords.push_back(P);
			P.x = pos.x + R * cos(i * inc) * cos((j+1)*inc2);
			P.y = pos.y + R * sin(i * inc);
			P.z = pos.z + R * cos(i * inc) * sin((j+1)*inc2);
			coords.push_back(P);

			nbNodesOfObstacle.push_back(4);

		}
	}

	//return nbSectors + 1 + 1;
	//FIXME: Working only as a circle for the moment
	//nbNodesOfObstacle.push_back(nbSectors);
	//return nbSectors + 1;
	return 0;
}
