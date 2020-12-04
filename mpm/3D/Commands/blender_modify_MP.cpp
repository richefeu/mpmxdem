//**********************
/*
checking using the 2D projections only! (mod in the future)
*/
//*********************

#include "blender_modify_MP.hpp"
#include "../Core/MPMbox.hpp"
#include "../ConstitutiveModels/ConstitutiveModel.hpp"

#include <factory.hpp>
static Registrar<Command, blender_modify_MP> registrar("blender_modify_MP");

void blender_modify_MP::read(std::istream & is)
{
	is >> groupNb >> modelName >> rho >> coordFile;
}

void blender_modify_MP::exec()
{
	importCoordinates();  // getting coordinates from text file

	auto itCM = box->models.find(modelName);
	if (itCM == box->models.end()) {
		std::cerr << "@blender_modify_MP::exec, model " << modelName << " not found" << std::endl;
	}
	ConstitutiveModel * CM = itCM->second;  //what is second?

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

		//getting all z pos of the material points
		std::set<double> zPositions;
		for (size_t j = 0; j < box->MP.size(); j++) {
			zPositions.insert(box->MP[j].pos.z);
		}
		//copying set to vector
		std::vector<double> zPos;
		std::copy(zPositions.begin(), zPositions.end(), std::back_inserter(zPos));

		double minMPz = zPos[0];
		double maxMPz = zPos[zPos.size() - 1];


		//FIXME: This check is def not the best but it's a way to start
		for (size_t p = 0; p < box->MP.size();p++) {
			if (box->MP[p].pos.z < maxMPz and box->MP[p].pos.z > minMPz) {
				vec2r point;
				point.x = box->MP[p].pos.x;
				point.y = box->MP[p].pos.y;
				if (pointinPolygon(point, verticePos) == true) {
					box->MP[p].constitutiveModel = CM;
					box->MP[p].groupNb = groupNb;
					box->MP[p].density = rho;
					box->MP[p].mass = box->MP[p].vol * rho;
					box->MP[p].q = box->MP[p].mass * box->MP[p].vel;
				}
			}
		}
	}
}

//TODO: should create a library and call it instead of implementing it here
bool blender_modify_MP::pointinPolygon(vec2r & point, std::vector<vec3r> & verticePos) {
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


void blender_modify_MP::importCoordinates() {
	//This allows to import multiple objects at the same time
	//Reads files generated with the new getCoord script (blender)

	char filepath[256];
	sprintf (filepath, "%s/%s", box->result_folder.c_str(), coordFile.c_str());

	std::ifstream infile(filepath);

	// std::ifstream infile(coordFile.c_str());
	// checking if it exists
	if (infile.good() != true){
		std::cerr << "@blender_modify_MP::importCoordinates, cannot open file " << std::endl;
		return;
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

void blender_modify_MP::findMax(std::vector<vec3r> const &allVertices
	, double& maxX, double& maxY, double& maxZ) {
	for (size_t i = 0; i < allVertices.size(); i++) {
		if (allVertices[i].x > maxX) maxX = allVertices[i].x;
		if (allVertices[i].y > maxY) maxY = allVertices[i].y;
		if (allVertices[i].z > maxZ) maxZ = allVertices[i].z;
	}
}

void blender_modify_MP::findMin(std::vector<vec3r> const &allVertices,
	double& minX, double& minY, double& minZ) {
	for (size_t i = 0; i < allVertices.size(); i++) {
		if (allVertices[i].x < minX) minX = allVertices[i].x;
		if (allVertices[i].y < minY) minY = allVertices[i].y;
		if (allVertices[i].z < minZ) minZ = allVertices[i].z;
	}
}

void blender_modify_MP::getSurface(std::vector<vec3r> const allVertices,
	std::vector<vec3r> &verticePos) {
	//I find the surface that is goint to be used in the
	//pointinPolygon algorithm. This time it'll be constant
	//but in the future we can have line equations between the
	//points that we get

	//sending one first point
	verticePos.push_back(allVertices[0]);
	//looping and checking for points with different "x,y" coordinates
	// FIXME: is that floorf function doing a good job? Sometimes coordinates from blender are 0.999999
	for (size_t i = 0; i < allVertices.size(); i++) {
		for (size_t j = 0; j < verticePos.size(); j++) {
			if (floorf(allVertices[i].x*1000)/1000
			== floorf(verticePos[j].x*1000)/1000 and
			floorf(allVertices[i].y*1000)/1000
			== floorf(verticePos[j].y*1000)/1000) {
				break;
			}
            // TODO: Maybe you should round the numbers before adding them (use a tolerance?)
			if (j == verticePos.size() - 1) verticePos.push_back(allVertices[i]);
		}
	}

	//sorting coordinates in clockwise order
	sortClockwise(verticePos);
}

void blender_modify_MP::sortClockwise(std::vector<vec3r> &verticePos) {
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
