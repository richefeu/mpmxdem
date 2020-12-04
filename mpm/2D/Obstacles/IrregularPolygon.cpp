#include "IrregularPolygon.hpp"
#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>
#include <fstream>
#include <iostream>

#include <factory.hpp>
static Registrar<Obstacle, IrregularPolygon> registrar("IrregularPolygon");

//1) THIS CLASS INHERITS DIRECTLY FROM THE POLYGON CLASS!
//2) You can move the polygon by moving the center only! (no need to adjust coordinates of vertices)
//3) YOU DONT REPEAT THE FIRST POINT AT THE END. ID DOES IT ON ITS OWN
void IrregularPolygon::read(std::istream & is)
{
	// if the centroid doesnt match the center given, the polygon
	// will be moved to match the one given by the user
	is >> group >> nVertices >> pos >>rot;
	std::string driveMode;
	is >> driveMode;
	// if (driveMode == "freeze") {
	// 	isFree = false;
	// 	vel.reset();
	// }
	//
	// else if (driveMode == "velocity") {
	// 	isFree = false;
	// 	is >> vel;
	// }
	double density;
	if (driveMode == "free") {
		isFree = true;
		is >> density;
		is >> vel >> vrot;
	}
	else { std::cerr << "@IrregularPolygon::read, driveMode " << driveMode << " is unknown!" << std::endl; }

	verticePos.clear();
	rot *= Mth::pi/180;  //convert to rad

	// reading the coordinates
	for (int i = 0; i < nVertices; i++) {
		vec2r point;
		is >> point;
		verticePos.push_back(point);
	}
	// adding first point to the end of vector (needed for centroid, area and vtk. not needed for inertia)
	verticePos.push_back(verticePos[0]);

	// calculating centroid
	vec2r centroid = calculateCentroid();

	// moving to center given by the user
	movePolygon(centroid);

	// inertia (needs to be calculated after moving the obstacle)
	I = calculateInertia(density);

	// calculating list of vectors from vertices to center(pos)
	for (size_t i = 0; i < verticePos.size(); i++) {
		vectorsCentertoVertices.push_back(verticePos[i] - pos);
	}
}

int IrregularPolygon::touch(MaterialPoint& MP, double& dn) {
  // it models the MP as a circle
  std::vector<vec2r> coords;
  // FIXME: this is duplicated code, should create a function out of it (also used in addVtkPoints)
  for (size_t i = 0; i < vectorsCentertoVertices.size(); i++) {
    double rotatedX = vectorsCentertoVertices[i].x * cos(rot) - vectorsCentertoVertices[i].y * sin(rot);
    double rotatedY = vectorsCentertoVertices[i].x * sin(rot) + vectorsCentertoVertices[i].y * cos(rot);
    vec2r rotatedVector(rotatedX, rotatedY);
    coords.push_back(pos + rotatedVector);
  }

  int Corner = -1;
  // vec2r point = MP.pos;
  double testdn;

  // part for adding the radius in the right direction
  vec2r c = pos - MP.pos;
  c.normalize();
  // vec2r c = c1.normalized();
  double len = sqrt(MP.vol) / 2.0;
  vec2r point = MP.pos + c * len;

  if (Polygon::pointinPolygon(nVertices, point, coords, MP, tang, normal, testdn)) {
    // FIXME: This if shouldnt exist since if its inside we assume that testdn is negative!
    // std::cout << "inside! testdn: "<<testdn << '\n';
    // std::cout << "touched!" << '\n';
    // double delvar;
    // std::cin >> delvar;
    if (testdn < 0.0) {
      dn = testdn;
      Corner = 1;
    }
  }
  return Corner;
}

void IrregularPolygon::checkProximity(MPMbox& MPM) {
  // TODO: needs to be adapted to the irregular polygons
  // std::cout << "polygon check proximity" << '\n';
  // Temporarily store the forces
  std::vector<Neighbor> Store = Neighbors;

  // Rebuild the list
  Neighbors.clear();
  Neighbor N;
  vec2r c;
  for (size_t p = 0; p < MPM.MP.size(); p++) {
    double sumSecurDistMin = MPM.MP[p].size;
    double sumSecurDist = MPM.MP[p].securDist + securDist;
    if (sumSecurDist < sumSecurDistMin) sumSecurDist = sumSecurDistMin;
    c = MPM.MP[p].pos - pos;
    // double dst = norm(c) - R;  //has to be adapted to the polygons but leave it like that for now
    // if (dst < sumSecurDist) {
    N.PointNumber = p;
    Neighbors.push_back(N);
    // }
  }

  // Get the known forces back
  size_t istore = 0;
  for (size_t inew = 0; inew < Neighbors.size(); inew++) {
    while (istore < Store.size() && Neighbors[inew].PointNumber < Store[istore].PointNumber) ++istore;
    if (istore == Store.size()) break;

    if (Store[istore].PointNumber == Neighbors[inew].PointNumber) {
      Neighbors[inew].fn = Store[istore].fn;
      Neighbors[inew].ft = Store[istore].ft;

      ++istore;
    }
  }
}

int IrregularPolygon::addVtkPoints(std::vector<vec2r>& coords) {
  coords.push_back(pos);  // center

  for (size_t i = 0; i < vectorsCentertoVertices.size(); i++) {
    double rotatedX = vectorsCentertoVertices[i].x * cos(rot) - vectorsCentertoVertices[i].y * sin(rot);
    double rotatedY = vectorsCentertoVertices[i].x * sin(rot) + vectorsCentertoVertices[i].y * cos(rot);
    vec2r rotatedVector(rotatedX, rotatedY);
    coords.push_back(pos + rotatedVector);
    // os << pos + rotatedVector << std::endl;
  }

  return nVertices + 1 + 1;  // 1 for the center + the extra initial point
}

// ******************
// Helper functions
// ******************

vec2r IrregularPolygon::calculateCentroid() {
  // https://en.wikipedia.org/wiki/Centroid
  double Cx = 0;
  double Cy = 0;
  for (size_t i = 0; i < verticePos.size() - 1; i++) {
    Cx += (verticePos[i].x + verticePos[i + 1].x) *
          (verticePos[i].x * verticePos[i + 1].y - verticePos[i + 1].x * verticePos[i].y);

    Cy += (verticePos[i].y + verticePos[i + 1].y) *
          (verticePos[i].x * verticePos[i + 1].y - verticePos[i + 1].x * verticePos[i].y);
  }
  double area = calculateArea();
  Cx = Cx / (6.0 * area);
  Cy = Cy / (6.0 * area);

  vec2r centroid(Cx, Cy);
  return centroid;
}

double IrregularPolygon::calculateArea() {
  double area = 0;
  for (size_t i = 0; i < verticePos.size() - 1; i++) {
    // std::cout << verticePos[i].x * verticePos[i + 1].y - verticePos[i + 1].x * verticePos[i].y << '\n';
    area += verticePos[i].x * verticePos[i + 1].y - verticePos[i + 1].x * verticePos[i].y;
  }
  return 0.5 * area;
}

void IrregularPolygon::movePolygon(vec2r& centroid) {
  // Moves the polygon if center given by the user is different from the centroid
  vec2r posDifference = pos - centroid;

  // modifying verticesPos
  for (size_t i = 0; i < verticePos.size(); i++) {
    verticePos[i] += posDifference;
  }
}

double IrregularPolygon::calculateInertia(double& density) {
  // https://en.wikipedia.org/wiki/List_of_moments_of_inertia
  double area = calculateArea();
  mass = area * 1 * density;

  std::vector<vec2r> myVector;
  // copying verticePos to a new vector (first value repeated at the end is not needed for this)
  for (size_t i = 0; i < verticePos.size() - 1; i++) {
    myVector.push_back(verticePos[i]);
  }

  // substracting the center...
  for (size_t i = 0; i < myVector.size(); i++) {
    myVector[i] -= pos;
  }

  double A = 0;
  double B = 0;

  for (size_t i = 0; i < myVector.size() - 1; i++) {
    double crossProduct = fabs(cross(myVector[i + 1], myVector[i]));

    A += crossProduct *
         ((myVector[i] * myVector[i]) + (myVector[i + 1] * myVector[i]) + (myVector[i + 1] * myVector[i + 1]));
    B += crossProduct;
  }
  double inertia = (1.0 / 6.0) * mass * (A / B);
  return inertia;
}
