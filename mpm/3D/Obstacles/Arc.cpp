#include "Arc.hpp"
#include <Core/MPMbox.hpp>

#include <../../common/factory.hpp>
static Registrar<Obstacle, Arc> registrar("Arc");
using namespace std;

void Arc::read(std::istream & is)
{
	is >> from >> lengthz >> angle >> R;
	
	//center of the circle that is used to create the arc
	center.x = from.x;
	center.y = from.y + R;
	center.z = from.z;
	vec3r u = from - center;

	angle = angle*Mth::pi/180; //converting back to radians
	anglep0 = atan2(u.y,u.x);  //angle of vector from center to "from"

}

bool Arc::touch(MaterialPoint & MP, int /*r*/, double & dn)
{
	center.z = MP.pos.z;
	vec3r c = MP.pos - center;
	double len = std::pow(MP.vol, 1/3.)/2; //because vol is size^2
	double angleMP = atan2(c.y,c.x);
	dn = R - norm(c) - len;
	if (dn<0.0 and (angleMP < anglep0 and angleMP > (anglep0 - angle))) {
		return true;
	}

	else return false;

}

void Arc::getContactFrame(MaterialPoint &, vec3r & /*N*/, vec3r & /*T1*/, vec3r & /*T2*/){}
void Arc::getContactData(MaterialPoint & MP, int /*r*/, double & vn, double & vt1, double & vt2, vec3r & N, vec3r & T1, vec3r & T2)
{

	//calculate normal and tangential vectors here
	center.z = MP.pos.z;
	vec3r c = MP.pos - center;
	c.normalize();  //this is the normal
	n = -c;

	t1.x = -n.y;
	t1.y = -n.x;
	t1.z = 0;

	t2.x = 0;
	t2.y = 0;
	t2.z = 1;

	vn = MP.vel * n;
	vt1 = MP.smoothVel * t1;
	vt2 = MP.smoothVel * t2;
	N=n;
	T1=t1;
	T2 = t2;

}

void Arc::checkProximity(MPMbox & MPM)
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

int Arc::addVtkPoints(std::vector<vec3r> & coords, std::vector<int> & nbNodesOfObstacle)
{

	const int nbSectors = 25;

	vec3r P;
	double inc = angle / (double)nbSectors;
	for (int i = 0 ; i < nbSectors ; i++) {
		P.x = center.x + R * cos(-i * inc + anglep0);
		P.y = center.y + R * sin(-i * inc + anglep0);
		P.z = 0;
		coords.push_back(P);

		P.x = center.x + R * cos(-i * inc + anglep0);
		P.y = center.y + R * sin(-i * inc + anglep0);
		P.z = lengthz;
		coords.push_back(P);

		P.x = center.x + R * cos(-(i + 1) * inc + anglep0);
		P.y = center.y + R * sin(-(i + 1) * inc + anglep0);
		P.z = lengthz;
		coords.push_back(P);

		P.x = center.x + R * cos(-(i + 1) * inc + anglep0);
		P.y = center.y + R * sin(-(i + 1) * inc + anglep0);
		P.z = 0	;
		coords.push_back(P);
		nbNodesOfObstacle.push_back(4);
	}
	return 0;
	//return 4*nbSectors;  //havent tested this part
	//return nbSectors * 4; //(+1?) should it be nbSectors*4 + 1*4?
	//return nbSectors + 1;
}
