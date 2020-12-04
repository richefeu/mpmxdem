#ifndef SPHERE2_HPP_C1EB5D8C
#define SPHERE2_HPP_C1EB5D8C

#include "blender_Obstacle.hpp"

struct Sphere2 : public blender_Obstacle
{
	double R;
	double len;
	vec3r n,t1, t2;
	std::vector<vec3r> rawVertices;
	std::vector<vec3r> correctedVertices;

	virtual void read(std::istream &is , std::string result_folder);
	virtual void checkProximity(MPMbox & MPM);
	virtual bool touch(MaterialPoint & MP, int r, double &dn);
	virtual void getContactFrame(MaterialPoint & MP, vec3r & n, vec3r & t1, vec3r & t2);
	virtual void getContactData(MaterialPoint & MP, int r, double & vn, double & vt1, double & vt2, vec3r & n, vec3r & t1, vec3r & t2);
	virtual int addVtkPoints(std::vector<vec3r> & coords, std::vector<int> & nbNodesOfObstacle);
};


#endif /* end of include guard: SPHERE2_HPP_C1EB5D8C */
