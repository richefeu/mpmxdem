#ifndef BLENDER_SET_MP_GRID_HPP_AAE2DD27
#define BLENDER_SET_MP_GRID_HPP_AAE2DD27

#include "Command.hpp"

struct blender_set_MP_grid : public Command
{
	void read(std::istream & is);
	void exec();

	bool pointinPolygon(vec2r & point, std::vector<vec3r> & verticePos);
	void importCoordinates();
	void findMax(std::vector<vec3r> const &verticePos,
		double& maxX, double& maxY, double& maxZ);

	void findMin(std::vector<vec3r> const &verticePos,
		double& minX, double& minY, double& minZ);

	void getSurface(std::vector<vec3r> const allVertices,
		std::vector<vec3r>& verticePos);

	void sortClockwise(std::vector<vec3r> &verticePos);


private:
	int groupNb;
	std::string modelName;
	std::string coordFile;
	double rho;
	double size;

	int nVertices;
	// std::vector<vec3r> verticePos; // the face (surface) that is used in the detection Algorithm
	// std::vector<vec3r> allVertices;
	std::vector<std::vector<vec3r>> container;  //holds different vectors that contain vertices for each object

};

#endif /* end of include guard: BLENDER_SET_MP_GRID_HPP_AAE2DD27 */
