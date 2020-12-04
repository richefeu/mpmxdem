#include "Plane.hpp"
#include <Core/MPMbox.hpp>

#include <../../common/factory.hpp>
static Registrar<Obstacle, Plane> registrar("Plane");

using namespace std;
//TODO: Make planes (or generally obstacles have their own properties. friction, Kn,...)
void Plane::read(std::istream & is)
{
	is >> group >> pos0 >> pos1 >> pos2 >> pos3;  //the plane is defined clockwise!
	//vec3r a = pos1 - pos0;
	//vec3r b = pos3 - pos0;
	t1 = pos1 - pos0; //careful here!!  Tangent in X direction
	t2 = pos3 - pos0; //Tangent in z direction
	t1.normalized();
	t2.normalized();
	n = cross(t1,t2);
	//std::cout<<n<<std::endl;
	n.normalized();

	n = -n;
}

bool Plane::touch(MaterialPoint & MP, int /*r*/, double &dn)
{
    vec3r c = MP.pos - pos0;
    double ct1 = c * t1;
    double ct2 = c * t2;

    //this "if" is to check the borders
    if ((0 < ct1 and ct1 < norm(pos1 - pos0)) and (0 < ct2 and  ct2 < norm(pos3 - pos0))) {
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
void Plane::getContactFrame(MaterialPoint &, vec3r & N, vec3r & T1, vec3r & T2){
	N = n;
	T1 = t1;
	T2 = t2;
}
void Plane::getContactData(MaterialPoint & MP, int , double & vn, double & vt1, double & vt2, vec3r & N, vec3r & T1, vec3r & T2)
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

void Plane::checkProximity(MPMbox & MPM)
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


int Plane::addVtkPoints(std::vector<vec3r> & coords, std::vector<int> & nbNodesOfObstacle)
{
	coords.push_back(pos0);
	coords.push_back(pos1);
	coords.push_back(pos2);
	coords.push_back(pos3);
	nbNodesOfObstacle.push_back(4);
	//return 4;
	return 0;
}
