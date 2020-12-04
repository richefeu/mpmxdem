#include "LeftBiplanar.hpp"
#include <Core/MPMbox.hpp>

#include <../../common/factory.hpp>
static Registrar<Obstacle, LeftBiplanar> registrar("LeftBiplanar");

using namespace std;
void LeftBiplanar::read(std::istream & is)
{

	is >> group >> from1 >> lengthz >> l1 >> l2 >> R >> angle;


	to1.x = from1.x - l1;
	to1.y = from1.y;
	to1.z = from1.z;
	
	//center of the circle that is used to create the arc
	center.x = to1.x;
	center.y = to1.y + R;
	center.z = to1.z;

	//arc
	vec3r u = to1 - center;
	angle = angle*Mth::pi/180; //converting back to radians
	anglep0 = atan2(u.y,u.x); //angle of vector u
	from2.x = center.x + R*cos(anglep0 - angle);
	from2.y = center.y + R*sin(anglep0 - angle);
	from2.z = from1.z;
	to2.x = from2.x - l2*cos(angle);
	to2.y = from2.y + l2*sin(angle);
	to2.z = from2.z;

	//settiing positions of planes (corners)
	pos0p1 = to1;
	pos1p1 = from1;
	pos2p1 = from1;
	pos2p1.z += lengthz;
	pos3p1 = to1;
	pos3p1.z += lengthz;

	pos0p2 = to2;
	pos1p2 = from2;
	pos2p2 = from2;
	pos2p2.z += lengthz;
	pos3p2 = to2;
	pos3p2.z += lengthz;


	//normal and tangent vectors for the planes
	t1p1 = pos1p1 - pos0p1;   //for plane number 1
	t1p1.normalize();
	t2p1 = pos3p1 - pos0p1;
	t2p1.normalize();
	t1p2 = pos1p2 - pos0p2;   //for plane number 2
	t1p2.normalize();
	t2p2 = pos3p2 - pos0p2;
	t2p2.normalize();
	n1 = cross(t1p1,t2p1);
	n1.normalized();
	n1 = -n1;
	n2 = cross(t1p2,t2p2);
	n2.normalized();
	n2 = -n2;

	//make a small cout to know where to place the box!!!!!
}

bool LeftBiplanar::touch(MaterialPoint & MP, int /*r*/, double &dn)
{
	center.z = MP.pos.z; //making sure the z is the same
	contactWith = 0;
	vec3r c1 = MP.pos - from1;
	vec3r c2 = MP.pos - from2;
	cArc = MP.pos - center;

	double len = std::pow(MP.vol, 1/3.)/2; //because vol is size^3
	double dst1 = c1 * n1 - len;
	double dst2 = c2 * n2 - len;
	double angleMP = atan2(cArc.y,cArc.x);
	double dstArc = R - norm(cArc) - len;
	cArc.normalize();
	nArc = -cArc;

	t1Arc.x = -nArc.y;  //check signs
	t1Arc.y = nArc.x;
	t1Arc.z = 0;

	t2Arc.x = 0;
	t2Arc.y = 0;
	t2Arc.z = 1;



	if (dstArc<0.0 and (angleMP < anglep0 and angleMP > (anglep0 - angle))) {  //for arc
		contactWith = 3;
		dn = dstArc;
		return true;
	}

	else if (dst1 < 0.0) { //for horizontal line
		dn = dst1;
		contactWith = 1;
		return true;
	}

	else if (dst2 < 0.0) { //for angled line
		dn = dst2;
		contactWith = 2;
		return true;
	}

	else return false;

}


void LeftBiplanar::getContactFrame(MaterialPoint &, vec3r & N, vec3r & T1, vec3r & T2)
{
	if (contactWith == 0) std::cerr << "@LeftBiplanar::getContactFrame, Problem with contact" << std::endl;

	if (contactWith == 1) {
		N = n1;
		T1 = t1p1;
		T2 = t2p1;
	}
	if (contactWith == 2) {
		N = n2;
		T1 = t1p2;
		T2 = t2p2;
	}
	if (contactWith == 3) {
		N=nArc;
		T1 = t1Arc;
		T2 = t2Arc;
	}

}
void LeftBiplanar::getContactData(MaterialPoint & MP, int /*r*/, double & vn, double & vt1, double & vt2, vec3r & N, vec3r & T1, vec3r & T2)
{

	if (contactWith == 0) cout<<"Problem with contact"<<endl;

	if (contactWith == 1) { //for horizontal line
		vn = MP.vel * n1;
		vt1 = MP.smoothVel * t1p1;
		vt2 = MP.smoothVel * t2p1;
		N = n1;
		T1 = t1p1;
		T2 = t2p1;
	}
	if (contactWith == 2) { //for angled line
		vn = MP.vel * n2;
		vt1 = MP.smoothVel * t1p2;
		vt2 = MP.smoothVel * t2p2;
		N = n2;
		T1 = t1p2;
		T2 = t2p2;
	}

	if (contactWith == 3) { //for arc
		vn = MP.vel * nArc;
		vt1 = MP.smoothVel * t1Arc;
		vt2 = MP.smoothVel * t2Arc;
		N=nArc;
		T1 = t1Arc;
		T2 = t2Arc;
	}

}

void LeftBiplanar::checkProximity(MPMbox & MPM)
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

	// Get the known forces back
	size_t istore = 0;
	for (size_t inew = 0 ; inew < Neighbors.size() ; inew++) {
		while (istore < Store.size() && Neighbors[inew].PointNumber < Store[istore].PointNumber) ++istore;
		if (istore == Store.size()) break;

		if (Store[istore].PointNumber == Neighbors[inew].PointNumber) {
			Neighbors[inew].fn[0] = Store[istore].fn[0];

			Neighbors[inew].ft[0] = Store[istore].ft[0];

			++istore;
		}
	}

}

int LeftBiplanar::addVtkPoints(std::vector<vec3r> & coords, std::vector<int> & nbNodesOfObstacle)
{
	//For Plane 1

	coords.push_back(pos0p1);
	coords.push_back(pos1p1);
	coords.push_back(pos2p1);
	coords.push_back(pos3p1);
	nbNodesOfObstacle.push_back(4);

	//For Arc
	const int nbSectors = 25;

	vec3r P;
	double inc = angle / (double)nbSectors;
	for (int i = 0 ; i <= nbSectors ; i++) {
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
	//nbPoints += nbSectors + 1;


	//for Plane 2
	coords.push_back(pos0p2);
	coords.push_back(pos1p2);
	coords.push_back(pos2p2);
	coords.push_back(pos3p2);
	nbNodesOfObstacle.push_back(4);

	return 0;
	//return (4 + 4*(nbSectors+1) + 4);  //FIXME: Not showing correcly in the vtk
	//return nbPoints;


}
