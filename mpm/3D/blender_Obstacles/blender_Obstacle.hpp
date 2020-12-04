#ifndef BLENDER_OBSTACLE_HPP_DEEAEE93
#define BLENDER_OBSTACLE_HPP_DEEAEE93

#include <iostream>
#include <vector>
#include <Core/Neighbor.hpp>
#include <math.h>
#include <../../common/vec2.hpp>
#include <../../common/vec3.hpp>

struct MPMbox;
struct MaterialPoint;
struct blender_Obstacle
{
	int group;
	std::string fileName;
	std::string obstacleName;
	double securDist;
	bool isFree;
	double mass;
	double I;
	vec3r pos;
	vec3r vel;
	vec3r acc;
	vec3r force;

	double rot;
	double vrot;
	double arot;
	double mom;
	int nbVertices;

	std::vector<Neighbor> Neighbors;

	virtual void read(std::istream & is, std::string result_folder) = 0;
	virtual void checkProximity(MPMbox & MPM) = 0;
	virtual bool touch(MaterialPoint & MP, int r, double &dn) = 0;
	virtual void getContactData(MaterialPoint & MP, int r, double & vn, double & vt1, double & vt2, vec3r & n, vec3r & t1, vec3r & t2) = 0;
	virtual void getContactFrame(MaterialPoint & MP, vec3r & n, vec3r & t1, vec3r & t2) = 0;
	virtual int addVtkPoints(std::vector<vec3r> & coords, std::vector<int> & nbNodesOfObstacle) = 0;
	//virtual int addVtkPoints(std::vector<vec3r> & coords) = 0;

	virtual bool MPisInside(MaterialPoint & MP);
	blender_Obstacle();
	virtual ~blender_Obstacle();
};

#endif /* end of include guard: BLENDER_OBSTACLE_HPP_DEEAEE93 */
