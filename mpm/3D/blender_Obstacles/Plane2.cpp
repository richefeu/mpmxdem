//this assumes square planes whose sides are parallel to the x and z axes

#include "Plane2.hpp"
#include <Core/MPMbox.hpp>

#include <../../common/factory.hpp>
static Registrar<blender_Obstacle, Plane2> registrar("Plane2");

using namespace std;

void Plane2::read(std::istream &is, std::string result_folder)
{
	is >> group >> fileName;

	char filepath[256];
	sprintf (filepath, "%s/%s", result_folder.c_str(), fileName.c_str());
	std::ifstream infile(filepath);

	// checking if it exists
	if (infile.good() != true){
		std::cerr << "@Plane2::read, cannot open boundaries file " << std::endl;
		exit(0);
	}

	infile >> obstacleName >> nbVertices;

	for (int i = 0; i < nbVertices; i++) {
		//is2 >> pos0 >> pos1 >> pos2 >> pos3;
		vec3r position;
		infile >> position;
		position.x = round(position.x*100)/100;
		position.y = round(position.y*100)/100;
		position.z = round(position.z*100)/100;
		rawVertices.push_back(position);
		// std::cout<<position<<std::endl;
	}
	// std::cout<<"\n"<<std::endl;
	// changing coordinate system (blender -> paraview)
	vec3r vertexCorrected;
	for (size_t i = 0; i < rawVertices.size(); i++) {
		vertexCorrected.x = rawVertices[i].x;
		vertexCorrected.y = rawVertices[i].z;
		vertexCorrected.z = rawVertices[i].y;

		correctedVertices.push_back(vertexCorrected);
	}


	//FIXME: This is prone to errors (if plane is flipped or smth)
	pos0 = correctedVertices[0];
	posX = correctedVertices[1];
	posZ = correctedVertices[2];
	posDiag = correctedVertices[3];

	t1 = posX - pos0;
	t2 = posZ - pos0;
	t1.normalized();
	t2.normalized();
	n = cross(t1,t2);
	n.normalized();
	n = -n;

}

bool Plane2::touch(MaterialPoint & MP, int /*r*/, double &dn)
{


    vec3r c = MP.pos - pos0;
    double ct1 = c * t1;
    double ct2 = c * t2;


    //this "if" is to check the borders
    if ((0 < ct1 and ct1 < norm(posX - pos0))
		and
		(0 < ct2 and ct2 < norm(posZ - pos0))) {

        vec3r c = MP.pos - pos0;
        double len = std::pow(MP.vol, 1/3.)/2;//  sqrt(MP.vol)/2.0;//because vol is size^3
        double dst = c * n - len;
        dn = dst;
        return (dst < 0.0);
    }

    else {
        return false;
    }
}
void Plane2::getContactFrame(MaterialPoint &, vec3r & N, vec3r & T1, vec3r & T2){
	N = n;
	T1 = t1;
	T2 = t2;

}
void Plane2::getContactData(MaterialPoint & MP, int , double & vn, double & vt1, double & vt2, vec3r & N, vec3r & T1, vec3r & T2)
{

	vn = MP.vel * n;
	vt1 = MP.smoothVel * t1;
	vt2 = MP.smoothVel * t2;
	//vt1 = MP.vel * t1;
	//vt2 = MP.vel * t2;
	N=n;
	T1 = t1;
	T2 = t2;

}

void Plane2::checkProximity(MPMbox & MPM)
{

	// Temporarily store the forces
	std::vector<Neighbor> Store = Neighbors;

	// Rebuild the list
	Neighbors.clear();
	Neighbor N;
	//double securDist = MPM.sizeMP * 100000000;
	//vec2r c;
	for (size_t p = 0 ; p < MPM.MP.size() ; p++) {
		//c = MPM.MP[p].pos - from;
		//double dstt = c * t;
		//if (dstt > -securDist && dstt < len + securDist) {
		//	double dstn = c * n;
		//	if (dstn < securDist) {
				N.PointNumber = p;
				Neighbors.push_back(N);
			//}
		//}
	}

	// Get the known forces back //this must be checked!!!
	size_t istore = 0;
	for (size_t inew = 0 ; inew < Neighbors.size() ; inew++) {
		while (istore < Store.size() && Neighbors[inew].PointNumber < Store[istore].PointNumber) ++istore;
		if (istore == Store.size()) break;

		if (Store[istore].PointNumber == Neighbors[inew].PointNumber) {
			Neighbors[inew].fn[0] = Store[istore].fn[0];
			Neighbors[inew].fn[1] = Store[istore].fn[1];
			Neighbors[inew].fn[2] = Store[istore].fn[2];
			Neighbors[inew].fn[3] = Store[istore].fn[3];

			Neighbors[inew].ft[0] = Store[istore].ft[0];
			Neighbors[inew].ft[1] = Store[istore].ft[1];
			Neighbors[inew].ft[2] = Store[istore].ft[2];
			Neighbors[inew].ft[3] = Store[istore].ft[3];

			++istore;
		}
	}

}


int Plane2::addVtkPoints(std::vector<vec3r> & coords, std::vector<int> & nbNodesOfObstacle)
{

	coords.push_back(pos0);
	coords.push_back(posX);
	coords.push_back(posDiag);
	coords.push_back(posZ);
	nbNodesOfObstacle.push_back(4);
	//return 4;
	return 0;


return 0;
}
