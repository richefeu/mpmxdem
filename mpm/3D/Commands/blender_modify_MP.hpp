#ifndef BLENDER_MODIFY_MP_HPP_AAE2DD27
#define BLENDER_MODIFY_MP_HPP_AAE2DD27

#include "Command.hpp"

struct blender_modify_MP : public Command
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

	int nVertices;
	std::vector<std::vector<vec3r>> container;  //holds different vectors that contain vertices for each object

};

#endif /* end of include guard: BLENDER_MODIFY_MP_HPP_AAE2DD27 */
