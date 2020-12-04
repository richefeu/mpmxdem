#ifndef ARC_HPP_4FE7E275
#define ARC_HPP_4FE7E275

#include "Obstacle.hpp"

struct Arc : public Obstacle
{
	vec3r from, to;
	double R, lengthz;
	vec3r center;
	double angle; //angle between vectors
	double anglep0; //angle of vector p0
	vec3r n,t,t1,t2;
	double delvar;

	virtual void read(std::istream & is);
	virtual void checkProximity(MPMbox & MPM);
	virtual bool touch(MaterialPoint & MP, int r, double &dn);
	virtual void getContactFrame(MaterialPoint & MP, vec3r & n, vec3r & t1, vec3r & t2);
	virtual void getContactData(MaterialPoint & MP, int r, double & vn, double & vt1, double & vt2, vec3r & n, vec3r & t1, vec3r & t2);
	virtual int addVtkPoints(std::vector<vec3r> & coords, std::vector<int> & nbNodesOfObstacle);
};

#endif /* end of include guard: ARC_HPP_4FE7E275 */
