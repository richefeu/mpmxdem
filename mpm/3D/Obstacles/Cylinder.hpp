#ifndef CYLINDER_HPP_C1EB5D8C
#define CYLINDER_HPP_C1EB5D8C

#include "Obstacle.hpp"

struct Cylinder : public Obstacle
{

	double R;
	double length;
	vec3r direction;
	vec3r unitDirection;
	double len;
	vec3r n,t1, t2;
	virtual void read(std::istream & is);
	virtual void deleteIfInside(MPMbox &MPM);
	virtual void checkProximity(MPMbox & MPM);
	virtual bool touch(MaterialPoint & MP, int r, double &dn);
	virtual void getContactFrame(MaterialPoint & MP, vec3r & n, vec3r & t1, vec3r & t2);
	virtual void getContactData(MaterialPoint & MP, int r, double & vn, double & vt1, double & vt2, vec3r & n, vec3r & t1, vec3r & t2);
	virtual int addVtkPoints(std::vector<vec3r> & coords, std::vector<int> & nbNodesOfObstacle);

};

#endif /* end of include guard: CYLINDER_HPP_C1EB5D8C */
