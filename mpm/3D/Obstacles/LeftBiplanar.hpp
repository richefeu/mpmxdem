#ifndef LEFTBIPLANAR_HPP_C1EB5D8D
#define LEFTBIPLANAR_HPP_C1EB5D8D

#include "Obstacle.hpp"
//#include "Line.hpp"
//#include "Arc.hpp"

//it could have been inherited from line and arc but i would have
//to make some changes to those classes. There's no time now

struct LeftBiplanar : public Obstacle
{

	vec3r from1, to1, from2, to2, center, cArc;
	vec3r n1, t1p1, t2p1,  nArc, t1Arc, t2Arc, n2, t1p2, t2p2; //t1p1  tangent 1 (x direction) plane 1
	vec3r pos0p1, pos1p1, pos2p1, pos3p1; //plane 1
	vec3r pos0p2, pos1p2, pos2p2, pos3p2; //plane 2

	double lengthz, l1, l2, R, angle, anglep0;
	int contactWith; // 1 line1, 2 line2, 3 arc
	int delvar;

	virtual void read(std::istream & is);
	virtual void checkProximity(MPMbox & MPM);
	virtual bool touch(MaterialPoint & MP, int r, double &dn);
	virtual void getContactFrame(MaterialPoint & MP, vec3r & n, vec3r & t1, vec3r & t2);
	virtual void getContactData(MaterialPoint & MP, int r, double & vn, double & vt1, double & vt2, vec3r & n, vec3r & t1, vec3r & t2);
	virtual int addVtkPoints(std::vector<vec3r> & coords, std::vector<int> & nbNodesOfObstacle);

};

#endif /* end of include guard: LEFTBIPLANAR_HPP_C1EB5D8D */
