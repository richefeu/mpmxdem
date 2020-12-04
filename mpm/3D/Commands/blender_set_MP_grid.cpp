//**********************
/*
Create figure in blender and export it using the script
This is a simple case and only works with planar 3D figues with equal faces.
That is, in the z direction there will be two equal faces
The command will get 2D projections (which will be the same) and fill
them with the help of the ray-casting algorithm.
It's not the most efficient but its made only once per simulation
*/
//*********************

#include "blender_set_MP_grid.hpp"
#include "../Core/MPMbox.hpp"
#include "../ConstitutiveModels/ConstitutiveModel.hpp"

#include <factory.hpp>
static Registrar<Command, blender_set_MP_grid> registrar("blender_set_MP_grid");



void blender_set_MP_grid::read(std::istream & is)
{
  // I should get the path to the blender file, groupNb, model name, rho and size
	// is >> groupNb >> modelName >> rho >> x0 >> y0 >> z0 >> x1 >> y1 >> z1 >> size;
	is >> groupNb >> modelName >> rho >> coordFile >> size;
}

void blender_set_MP_grid::exec()
{
	if (box->Grid.lx / size > 3.0 || box->Grid.ly / size > 3.0) {
		std::cerr << "@blender_set_MP_grid::exec, Check Grid size - MP size ratio" << std::endl;
		exit(0);
	}

	importCoordinates();  // getting coordinates from text file

	auto itCM = box->models.find(modelName);
	if (itCM == box->models.end()) {
		std::cerr << "@blender_set_MP_grid::exec, model " << modelName << " not found" << std::endl;
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

	// we find min and max in each direction
	// That is to create a box around our polygon
	//and fill only where the pointinPolygon func is true
	double inf = 1000000;
	double neginf = -1000000;

	for (size_t i = 0; i < container.size(); i++) {

		std::vector<vec3r> allVertices = container[i];
		std::vector<vec3r> verticePos;
		// std::cout<<"allVertices.size(): "<<allVertices.size()<<std::endl;
		double maxX = neginf, maxY = neginf, maxZ = neginf;
		findMax(allVertices, maxX, maxY, maxZ);

		double minX = inf, minY= inf, minZ = inf;  // Assuming that no neg values are acceptable
		findMin(allVertices, minX, minY, minZ);

		getSurface(allVertices, verticePos);

		double half = 0.5 * size;
		for (double z = minZ + half; z <= maxZ - half; z +=size) {
			for (double y = minY + half ; y <= maxY - half ; y += size) {
				for (double x = minX + half ; x <= maxX - half ; x += size) {

					P.pos.x = x;
					P.pos.y = y;
					P.pos.z = z;

					vec2r point;
					point.x = x;
					point.y = y;

					if (pointinPolygon(point, verticePos) == true) {
						CM->initializeMaterialPoint(P); /// check if this is a problem
						box->MP.push_back(P);
					}
				}
			}
		}
	}

	// checking for MP outside the grid before the start of the simulation
	for (size_t p = 0; p < box->MP.size(); p++) {
		if(box->MP[p].pos.x > box->Grid.Nx*box->Grid.lx or  box->MP[p].pos.x < 0.0
		or box->MP[p].pos.y > box->Grid.Ny*box->Grid.ly or  box->MP[p].pos.y < 0.0
		or box->MP[p].pos.z > box->Grid.Nz*box->Grid.lz or  box->MP[p].pos.z < 0.0)
		{
			std::cerr << "@blender_set_MP_grid::exec, Check before simulation: Some MPs are not inside the grid"<< std::endl;
			exit(0);
		}
	}

	std::cout<<"@blender_set_MP_grid::exec, total number of MPs: "<<box->MP.size()<<std::endl;

}

//TODO: should create a library and call it instead of implementing it here
bool blender_set_MP_grid::pointinPolygon(vec2r & point, std::vector<vec3r> & verticePos) {
	int nVertices = verticePos.size(); //because the last vertex is the same as the first
	int i, j = 0;
	bool c = false;

	//from http://stackoverflow.com/questions/217578/how-can-i-determine-whether-a-2d-point-is-within-a-polygon/2922778#2922778    --  answer by nirg
	for (i = 0, j = nVertices-1; i < nVertices; j = i++) {
		//TODO: In the future remove point. use the MP and r instead
		if ( ((verticePos[i].y>point.y) != (verticePos[j].y>point.y))
		&& (point.x < (verticePos[j].x-verticePos[i].x) * (point.y-verticePos[i].y)
		/ (verticePos[j].y-verticePos[i].y) + verticePos[i].x) ) {
			//contact is detected very precisely though!
			c = !c;
		}
	}
	return c;
}


void blender_set_MP_grid::importCoordinates() {
	//This allows to import multiple objects at the same time
	//Reads files generated with the new getCoord script (blender)
	char filepath[256];
	sprintf (filepath, "%s/%s", box->result_folder.c_str(), coordFile.c_str());

	// std::ifstream infile(coordFile.c_str());
	std::ifstream infile(filepath);

	// checking if it exists
	if (infile.good() != true){
		std::cerr << "@blender_set_MP_grid::importCoordinates, cannot open MPcoordinates file " << std::endl;
		exit(0);
	}

	// reading from text file
	std::string token;
	infile >> token;


	while(!infile.eof()) {
		// std::cout<<"token: "<<token<<std::endl;
		if (token == "Object") {
			std::vector<vec3r> rawVertices;
			std::vector<vec3r> correctedVertices;
			int nbVertices;
			infile >> nbVertices;
			double a, b, c;
			vec3r singleCoordinates;
			for (int i = 0; i < nbVertices; i++) {
				infile >> a >> b >> c;
				singleCoordinates.x = a;
				singleCoordinates.y = b;
				singleCoordinates.z = c;
				// std::cout<<singleCoordinates<<std::endl;
				rawVertices.push_back(singleCoordinates);
			}

			// changing coordinate system (blender -> paraview)
			vec3r vertexCorrected;
			for (size_t i = 0; i < rawVertices.size(); i++) {
				vertexCorrected.x = rawVertices[i].x;
				vertexCorrected.y = rawVertices[i].z;
				vertexCorrected.z = rawVertices[i].y;

				correctedVertices.push_back(vertexCorrected);
			}
			container.push_back(correctedVertices);
		}
		infile >> token;
	}
}

void blender_set_MP_grid::findMax(std::vector<vec3r> const &allVertices
	, double& maxX, double& maxY, double& maxZ) {
	for (size_t i = 0; i < allVertices.size(); i++) {
		if (allVertices[i].x > maxX) maxX = allVertices[i].x;
		if (allVertices[i].y > maxY) maxY = allVertices[i].y;
		if (allVertices[i].z > maxZ) maxZ = allVertices[i].z;
	}
}

void blender_set_MP_grid::findMin(std::vector<vec3r> const &allVertices,
	double& minX, double& minY, double& minZ) {
	for (size_t i = 0; i < allVertices.size(); i++) {
		if (allVertices[i].x < minX) minX = allVertices[i].x;
		if (allVertices[i].y < minY) minY = allVertices[i].y;
		if (allVertices[i].z < minZ) minZ = allVertices[i].z;
	}
}

void blender_set_MP_grid::getSurface(std::vector<vec3r> const allVertices,
	std::vector<vec3r> &verticePos) {
	//I find the surface that is goint to be used in the
	//pointinPolygon algorithm. This time it'll be constant
	//but in the future we can have line equations between the
	//points that we get

	//sending one first point
	verticePos.push_back(allVertices[0]);
	//looping and checking for points with different "x,y" coordinates
	// TODO: is that floorf function doing a good job? Sometimes coordinates from blender are 0.999999
	for (size_t i = 0; i < allVertices.size(); i++) {
		for (size_t j = 0; j < verticePos.size(); j++) {
			if (floorf(allVertices[i].x*1000)/1000
			== floorf(verticePos[j].x*1000)/1000 and
			floorf(allVertices[i].y*1000)/1000
			== floorf(verticePos[j].y*1000)/1000) {
				break;
			}
            // TODO: Maybe you should round the numbers before adding them
			if (j == verticePos.size() - 1) verticePos.push_back(allVertices[i]);
		}
	}

	//sorting coordinates in clockwise order
	sortClockwise(verticePos);
}

void blender_set_MP_grid::sortClockwise(std::vector<vec3r> &verticePos) {
	//find an average center of the figure and use it to calculate the angles.
	//then sort accordingly
	//FIXME:the center is just to have the coord of a point inside the figure

	// 1) getting avg center
	vec2r center;
	double sumX = 0.0 , sumY = 0.0;
	for (size_t i = 0; i < verticePos.size(); i++) {
		sumX += verticePos[i].x;
		sumY += verticePos[i].y;
	}
	center.x = sumX/static_cast<double>(verticePos.size());
	center.y = sumY/static_cast<double>(verticePos.size());

	// 2) filling vector with angles
	std::vector<double> angles;
	for (size_t i = 0; i < verticePos.size(); i++) {
		vec2r vect;
		vect.x = verticePos[i].x - center.x;
		vect.y = verticePos[i].y - center.y;
		double angle = atan2(vect.y,vect.x)*180/Mth::pi;
		if (angle < 0) angle += 360;
		angles.push_back(angle);
	}

	// 3) sorting using the angles vector (sorting manually...should be improved)
	// using the angles vector we sort the verticePos vector
	// I used insertion sort!
	int j;
	double temp;
	vec3r tempVertex;
	for (size_t i = 1; i < verticePos.size(); i++){
		temp = angles[i];
		tempVertex = verticePos[i];
		j = i - 1;
		while ((temp < angles[j]) && (j >= 0)) {
			angles[j + 1] = angles[j];
			verticePos[j + 1] = verticePos[j];
			j = j - 1;
		}
		angles[j + 1] = temp;
		verticePos[j + 1] = tempVertex;
	}

	// 4) adding first point to the end of the vector (no need)
	// verticePos.push_back(verticePos[0]);
}
