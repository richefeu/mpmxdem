#include "Polygon.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

std::string Polygon::getRegistrationName() { return std::string("Polygon"); }

// UPDATE March 2018: It doesnt work with corners anymore.
// The way we find the edge of the MP should prob be improved, although i think is not "that" speculative

void Polygon::read(std::istream& is) {

  is >> group >> nVertices >> pos >> rot >> R;

  std::string driveMode;
  is >> driveMode;
  if (driveMode == "freeze") {
    isFree = false;
    vel.reset();
  } else if (driveMode == "velocity") {
    isFree = false;
    is >> vel;
  } else if (driveMode == "free") {
    isFree = true;
    double density;
    is >> density;
    double area = Area();
    mass = area * 1 * density;
    double sideLength = norm(verticePos[0] - verticePos[1]);
    I = mass * (sideLength * sideLength + sideLength * sideLength) / 12;  // works for square only
    is >> vel >> vrot;
  } else {
    std::cerr << "@Polygon::read, driveMode " << driveMode << " is unknown!" << std::endl;
  }

  verticePos.clear();
  rot *= Mth::pi / 180.0;  // convert to rad
  createPolygon(verticePos);
}

void Polygon::write(std::ostream& os) {
  os << group << ' ' << nVertices << ' ' << pos << ' ' << rot << ' ' << R << ' ';
  if (isFree == false) {
    os << "velocity " << vel << '\n';
  } else {
    double density = mass / (Mth::pi * R * R); // FAKE !!!!!
    os << "free " << density << ' ' << vel << ' ' << vrot << '\n';
  }
}

int Polygon::touch(MaterialPoint& MP, double& dn) {
  // it models the MP as a circle
  verticePos.clear();
  createPolygon(verticePos);
  int Corner = -1;
  double testdn;

  // part for adding the radius in the right direction
  vec2r c = pos - MP.pos;
  c.normalize();
  // vec2r c = c1.normalized();
  double len = sqrt(MP.vol) / 2.0;
  vec2r point = MP.pos + c * len;

  if (pointinPolygon(point, MP, testdn)) {
    // FIXME: This if shouldnt exist since if its inside we assume that testdn is negative!
    if (testdn < 0.0) {
      dn = testdn;
      Corner = 1;
    }
  }
  return Corner;
}

void Polygon::getContactFrame(MaterialPoint&, vec2r& N, vec2r& T) {
  N = normal;
  T = tang;
}

void Polygon::checkProximity(MPMbox& MPM) {
  // TODO: needs to be adapted to the polygons (it works as if it was a circle!)
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
    if (sumSecurDist < sumSecurDistMin) {sumSecurDist = sumSecurDistMin;}
    c = MPM.MP[p].pos - pos;
    double dst = norm(c) - R;  // has to be adapted to the polygons but leave it like that for now
    if (dst < sumSecurDist) {
      N.PointNumber = p;
      Neighbors.push_back(N);
    }
  }

  // Get the known forces back
  size_t istore = 0;
  for (size_t inew = 0; inew < Neighbors.size(); inew++) {
    while (istore < Store.size() && Neighbors[inew].PointNumber < Store[istore].PointNumber) {++istore;}
    if (istore == Store.size()) {break;}

    if (Store[istore].PointNumber == Neighbors[inew].PointNumber) {
      Neighbors[inew].fn = Store[istore].fn;
      Neighbors[inew].ft = Store[istore].ft;

      ++istore;
    }
  }
}


// not being used
bool Polygon::inside(vec2r& x) {
  vec2r l = x - pos;
  return (norm2(l) < R * R);
}

// TODO: shoudl create a library and call it instead of implementing it here
bool Polygon::pointinPolygon(vec2r& point, MaterialPoint& MP,
                             double& testdn) {
  int i, j = 0;
  bool c = false;

  // from
  // http://stackoverflow.com/questions/217578/how-can-i-determine-whether-a-2d-point-is-within-a-polygon/2922778#2922778
  // --  answer by nirg

  for (i = 0, j = nVertices - 1; i < nVertices; j = i++) {
    // TODO: In the future remove point. use the MP and r instead
    if (((verticePos[i].y > point.y) != (verticePos[j].y > point.y)) &&
        (point.x <
         (verticePos[j].x - verticePos[i].x) * (point.y - verticePos[i].y) / (verticePos[j].y - verticePos[i].y) +
             verticePos[i].x)) {
      c = !c;
    }
  }
  if (c == true) {  // if there is contact we find normal and tangent.
    // FIXME: this assumes that contact only takes place on top of the MP!
    tang = MP.corner[2] - MP.corner[3];
    tang.normalize();
    normal.x = tang.y;
    normal.y = -tang.x;

    // finding the closest vertex...
    // FIXME: this part is to be reviewed
    size_t closestVertex = 0;
    double distanceClosest = 10000000;                // just giving a random initial val
    for (size_t e = 0; e < verticePos.size(); e++) {  // finding the vertex  that is in contact
      double normDiff = norm(point - verticePos[e]);

      if (normDiff < distanceClosest) {
        distanceClosest = normDiff;
        closestVertex = e;
      }
    }
    testdn = (verticePos[closestVertex] - point) * -normal;
  }
  return c;
}

double Polygon::Area() {
  createPolygon(verticePos);
  // creating a list with the first point appended at the end (in order to make it easier)
  std::vector<vec2r> vectorArea(verticePos.begin(), verticePos.end());
  vectorArea.push_back(verticePos[0]);
  double sum = 0;
  for (size_t i = 0; i < verticePos.size() - 1; i++) {
    sum += determinant(vectorArea[i], vectorArea[i + 1]);
  }
  // std::cout<<0.5*sum<<std::endl;
  return 0.5 * sum;
}

void Polygon::createPolygon(std::vector<vec2r>& vect) {

  vec2r P;
  double inc = Mth::_2pi / (double)nVertices;
  for (int i = 0; i <= nVertices; i++) {
    P.x = pos.x + R * cos(rot + i * inc);
    P.y = pos.y + R * sin(rot + i * inc);
    vect.push_back(P);
  }
}
