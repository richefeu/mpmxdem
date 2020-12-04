//**********************
/*
Export ascii stl using blender
Reads the vertices and stores them (triangles)
Uses different function to check if inside the
polygon made by the triangles
*/
//*********************

#include "blender_set_MP_grid2.hpp"
// #include "../Core/MPMbox.hpp"
#include "../ConstitutiveModels/ConstitutiveModel.hpp"
#include "../../../common/geoTool.hpp"


#include <factory.hpp>
static Registrar<Command, blender_set_MP_grid2> registrar("blender_set_MP_grid2");

void blender_set_MP_grid2::read(std::istream & is)
{
  	is >> groupNb >> modelName >> rho >> stlFile >> size;
}

void blender_set_MP_grid2::exec()
{
	if (box->Grid.lx / size > 3.0 || box->Grid.ly / size > 3.0) {
		std::cerr << "@blender_set_MP_grid2::exec, Check Grid size - MP size ratio" << std::endl;
		exit(0);
	}

	importCoordinates();  // getting coordinates from stl file

	//finding max and min values
	// That is to create a box around our polygon
	//and fill only where the pointinPolygon func is true
	double inf = 1000000;
	double neginf = -1000000;
	vec3r max(neginf,neginf,neginf);
	findMax(max);

	vec3r min(inf, inf, inf);  // Assuming that no neg values are acceptable
	findMin(min);

	auto itCM = box->models.find(modelName);
	if (itCM == box->models.end()) {
		std::cerr << "@blender_set_MP_grid2::exec, model " << modelName << " not found" << std::endl;
	}
	ConstitutiveModel * CM = itCM->second;  //What is second?

	MaterialPoint P;
	P.groupNb = groupNb;
	P.vol = size * size * size;
	P.density = rho;
	P.mass = P.vol * rho;
	P.vel.reset();
	P.size = size;

	P.stress.xx = P.stress.xy = P.stress.xz = P.stress.yx = P.stress.yy = P.stress.yz = P.stress.zx = P.stress.zy = P.stress.zz = 0.0;
	P.strain.xx = P.strain.xy = P.strain.xz = P.strain.yx = P.strain.yy = P.strain.yz = P.strain.zx = P.strain.zy = P.strain.zz = 0.0;
	P.plasticStress.xx = P.plasticStress.xy = P.plasticStress.xz = P.plasticStress.yx = P.plasticStress.yy = P.plasticStress.yz = P.plasticStress.zx = P.plasticStress.zy = P.plasticStress.zz = 0.0;
	P.plasticStrain.xx = P.plasticStrain.xy = P.plasticStrain.xz = P.plasticStrain.yx = P.plasticStrain.yy = P.plasticStrain.yz = P.plasticStrain.zx = P.plasticStrain.zy = P.plasticStrain.zz = 0.0;

	P.constitutiveModel = CM;
	P.q = P.mass * P.vel; ///change
	P.splitCount = 0;


	double half = 0.5 * size;
	for (double z = min.z + half; z <= max.z - half; z +=size) {
		for (double y = min.y + half ; y <= max.y - half ; y += size) {
			for (double x = min.x + half ; x <= max.x - half ; x += size) {

				P.pos.x = x;
				P.pos.y = y;
				P.pos.z = z;

				vec3r point;
				point.x = x;
				point.y = y;
				point.z = z;

				if (pointinPolygon(point) == true) {
					CM->initializeMaterialPoint(P);
					box->MP.push_back(P);
				}
			}
		}
	}
	std::cout<<"@blender_set_MP_grid2::exec, total number of MPs: "<<box->MP.size()<<std::endl;
}

void blender_set_MP_grid2::importCoordinates() {
	char filepath[256];
	sprintf (filepath, "%s/%s", box->result_folder.c_str(), stlFile.c_str());

	std::ifstream infile(filepath);

	// checking if it exists
	if (infile.good() != true){
		std::cerr << "@blender_set_MP_grid::importCoordinates, cannot open MPcoordinates file " << std::endl;
		exit(0);
	}

	// reading from text file
	std::string token;
	infile >> token;
    std::vector<vec3r> rawVertices;
	while(!infile.eof()) {

		if (token == "vertex") {
            vec3r vertex;
			infile >> vertex;
			rawVertices.push_back(vertex);
		}

		if (rawVertices.size() == 3){
            // std::cout<<"vertices = 3"<<std::endl;
			//FIXME: it means when a triangle is completed (bug prone...)
			// changing coordinate system (blender -> paraview)
			vec3r vertexCorrected;
			std::vector<vec3r> correctedVertices;
			for (size_t i = 0; i < rawVertices.size(); i++) {
				vertexCorrected.x = rawVertices[i].x;
				vertexCorrected.y = rawVertices[i].z;
				vertexCorrected.z = rawVertices[i].y;

				correctedVertices.push_back(vertexCorrected);
			}
			faces.push_back(correctedVertices);
            rawVertices.clear();
		}
		infile >> token;
	}
}

void blender_set_MP_grid2::findMax(vec3r & max) {
	for (size_t f = 0; f < faces.size(); f++) {
		for (size_t v = 0; v < faces[f].size(); v++) {
			if (faces[f][v].x > max.x) max.x = faces[f][v].x;
			if (faces[f][v].y > max.y) max.y = faces[f][v].y;
			if (faces[f][v].z > max.z) max.z = faces[f][v].z;
		}
	}
}

void blender_set_MP_grid2::findMin(vec3r & min) {
	for (size_t f = 0; f < faces.size(); f++) {
		for (size_t v = 0; v < faces[f].size(); v++) {
			if (faces[f][v].x < min.x) min.x = faces[f][v].x;
			if (faces[f][v].y < min.y) min.y = faces[f][v].y;
			if (faces[f][v].z < min.z) min.z = faces[f][v].z;
		}
	}
}

//TODO: should create a library and call it instead of implementing it here
bool blender_set_MP_grid2::pointinPolygon(vec3r & point) {

	vec3r v1, v2, v3;
  	size_t nb_intersect = 0;
  	for (size_t f = 0; f < faces.size(); ++f) {
    	v1 = faces[f][0];

        for (size_t v = 1; v < faces[f].size() - 1; v++){
    	    v2 = faces[f][v];
  		    v3 = faces[f][v+1];

  		    if (geoTool::intersectTriangle(point, vec3r::unit_x(), v1, v2, v3) > 0) {
    		    ++(nb_intersect);
    		    break;
  		    }
        }
  	}
  	if (nb_intersect % 2 != 0) return true;  // Should be optimized by the compiler
	else return false;

}
