//this assumes square planes whose sides are parallel to the x and z axes

#include "Sphere2.hpp"
#include <Core/MPMbox.hpp>

#include <../../common/factory.hpp>
static Registrar<blender_Obstacle, Sphere2> registrar("Sphere2");

using namespace std;

void Sphere2::read(std::istream &is, std::string result_folder)
{
	is >> group >> fileName;

	char filepath[256];
	sprintf (filepath, "%s/%s", result_folder.c_str(), fileName.c_str());
	std::ifstream infile(filepath);

	// checking if it exists
	if (infile.good() != true){
		std::cerr << "@Sphere2::read, cannot open boundaries file " << std::endl;
		exit(0);
	}

	infile >> obstacleName >> R >> pos;

	// changing coordinate system (blender -> paraview)
	double temp = pos.y;
	pos.y = pos.z;
	pos.z = temp;

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

bool Sphere2::touch(MaterialPoint & MP, int /*r*/, double &dn)
{
	vec3r c = MP.pos - pos;
	double len = std::pow(MP.vol, 1/3.)/2;//  sqrt(MP.vol)/2.0;//because vol is size^3

    dn = norm(c) - R - len;

	if (dn < 0.0) return true;
	else return false;
}

void Sphere2::getContactFrame(MaterialPoint &MP, vec3r & N, vec3r & T1, vec3r & T2)
{
	N = MP.pos - pos;
	N.normalize();
	T1.x = -N.y;
	T1.y = N.x;
	T1.z = 0.0;
	T2.x = 0.0;
	T2.y = N.z;
	T2.z = -N.y;
}

void Sphere2::getContactData(MaterialPoint & /*MP*/, int , double & /*vn*/, double & /*vt1*/, double & /*vt2*/, vec3r & /*N*/, vec3r & /*T1*/, vec3r & /*T2*/)
{

}

void Sphere2::checkProximity(MPMbox & MPM)
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


int Sphere2::addVtkPoints(std::vector<vec3r> & coords, std::vector<int> & nbNodesOfObstacle)
{
	const int nbSectors = 75;
	const int nbSectors2 = 75;
	vec3r P;
	double inc = Mth::_2pi / (double) nbSectors;
	double inc2 = Mth::pi/(double) nbSectors2;

	// std::cout<<"center: "<<pos<<std::endl;
	for (int j = 0; j < nbSectors2; j++) {
		for (int i = 0 ; i < nbSectors ; i++) {
			P.x = pos.x + R * cos(i * inc) * cos(j*inc2);
			P.y = pos.y + R * sin(i * inc);
			P.z = pos.z + R * cos(i * inc) * sin(j*inc2);
			// std::cout<<"point1: "<<P<<std::endl;
			coords.push_back(P);
			P.x = pos.x + R * cos((i+1) * inc) * cos(j*inc2);
			P.y = pos.y + R * sin((i+1) * inc);
			P.z = pos.z + R * cos((i+1) * inc) * sin(j*inc2);
			// std::cout<<"point2: "<<P<<std::endl;
			coords.push_back(P);
			P.x = pos.x + R * cos((i+1) * inc) * cos((j+1)*inc2);
			P.y = pos.y + R * sin((i+1) * inc);
			P.z = pos.z + R * cos((i+1) * inc) * sin((j+1)*inc2);
			// std::cout<<"point3: "<<P<<std::endl;
			coords.push_back(P);
			P.x = pos.x + R * cos(i * inc) * cos((j+1)*inc2);
			P.y = pos.y + R * sin(i * inc);
			P.z = pos.z + R * cos(i * inc) * sin((j+1)*inc2);
			// std::cout<<"point4: "<<P<<"\n"<<std::endl;
			coords.push_back(P);

			nbNodesOfObstacle.push_back(4);
		}
	}
	return 0;
}
