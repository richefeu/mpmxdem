#ifndef BLENDER_SET_MP_GRID2_HPP_AAE2DD27
#define BLENDER_SET_MP_GRID2_HPP_AAE2DD27

#include "Command.hpp"
#include "../Core/MPMbox.hpp"

struct blender_set_MP_grid2 : public Command
{
	void read(std::istream & is);
	void exec();

	void importCoordinates();
	void findMax(vec3r &max);
	void findMin(vec3r &min);
	bool pointinPolygon(vec3r & point);

private:
	int groupNb;
	std::string modelName;
	std::string stlFile;
	double rho;
	double size;

	std::vector<std::vector<vec3r>> faces;  // each vector holds the vertices of a face

};

#endif /* end of include guard: BLENDER_SET_MP2_GRID_HPP_AAE2DD27 */
