#include "Polygon.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

#include "factory.hpp"
static Registrar<Obstacle, Polygon> registrar("Polygon");
// UPDATE March 2018: It doesnt work with corners anymore.
// The way we find the edge of the MP should prob be improved, although i think is not "that" speculative

void Polygon::read(std::istream& is) {

  // std::cout<<"enters read!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
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
    std::cerr << "@Circle::read, driveMode " << driveMode << " is unknown!" << std::endl;
  }

  verticePos.clear();
  rot *= Mth::pi / 180.0;  // convert to rad
  createPolygon(verticePos);
}

int Polygon::touch(MaterialPoint& MP, double& dn) {
  // std::cout << "Touch polygon" << '\n';
  // it models the MP as a circle
  verticePos.clear();
  createPolygon(verticePos);
  int Corner = -1;
  // vec2r point = MP.pos;
  double testdn;

  // part for adding the radius in the right direction
  vec2r c = pos - MP.pos;
  c.normalize();
  // vec2r c = c1.normalized();
  double len = sqrt(MP.vol) / 2.0;
  vec2r point = MP.pos + c * len;

  if (pointinPolygon(nVertices, point, verticePos, MP, tang, normal, testdn)) {
    // FIXME: This if shouldnt exist since if its inside we assume that testdn is negative!
    if (testdn < 0.0) {
      dn = testdn;
      Corner = 1;
    }
  }
  return Corner;

  // original (uses the corners)
  // TODO: It only works when contact happens on the upper part of the MP. Should be corrected soon!!
  // A more advanced version of Polygon.
  // It includes MP corner inside polygon, but also polygon corner inside MP

  /*
  verticePos.clear();
  createPolygon(verticePos);

  int Corner = -1;
  double testdn;

  vec2r point = MP.corner[0];


  //Checking if one of the MP corners is inside the polygon
  if (pointinPolygon(nVertices, point, verticePos, MP, tang, normal, testdn)) {
          Corner = 0;
          dn = testdn;
  }

  for (int r = 1; r < 4 ; r++) {
          vec2r point = MP.corner[r];
          if (pointinPolygon(nVertices, point, verticePos, MP, tang, normal, testdn)) {

                  if (testdn < dn) {  //dn is supposed to be a negative value
                          Corner = r;
                          dn = testdn;
                  }
          }
  }

  if (Corner >= 0) {
          return Corner;  //if contact found with corner, exit function
  }

  else {
          //TODO: This second part should actually go first since its more likely to happen. It would accelerate the
code a bit
          // If not found yet...
          // we check if one of the polygon edges is
          //inside the MP (MP bounded by the four corners)
          int closestCorner;
          if (edgeinMP(verticePos[0], closestCorner, MP, tang, normal, testdn)){
                  Corner = closestCorner; //i need it for later (MPMbox.cpp)
                  dn = testdn;
          }

          for (int e = 1; e < nVertices; e++){
                  vec2r edgePos = verticePos[e];
                  if(edgeinMP(edgePos, closestCorner, MP, tang, normal, testdn)) {
                          if (testdn < dn) {
                                  Corner = closestCorner; //i need it for later (MPMbox.cpp)
                                  dn = testdn;
                          }
                  }
          }
}

  return Corner;
  */
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
    if (sumSecurDist < sumSecurDistMin) sumSecurDist = sumSecurDistMin;
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
    while (istore < Store.size() && Neighbors[inew].PointNumber < Store[istore].PointNumber) ++istore;
    if (istore == Store.size()) break;

    if (Store[istore].PointNumber == Neighbors[inew].PointNumber) {
      Neighbors[inew].fn = Store[istore].fn;
      Neighbors[inew].ft = Store[istore].ft;

      ++istore;
    }
  }
}

int Polygon::addVtkPoints(std::vector<vec2r>& coords) {
  coords.push_back(pos);  // center
  createPolygon(coords);
  return nVertices + 1 + 1;
}

// not being used
bool Polygon::MPisInside(MaterialPoint& MP) {
  vec2r l = MP.pos - pos;
  double radiusMP = 0.5 * MP.vol;
  double sumR = (R + radiusMP);
  return (norm2(l) < sumR * sumR);
}

// TODO: shoudl create a library and call it instead of implementing it here
bool Polygon::pointinPolygon(int& nVertices, vec2r& point, std::vector<vec2r>& verticePos, MaterialPoint& MP,
                             vec2r& tang, vec2r& normal, double& testdn) {
  int i, j = 0;
  bool c = false;
  // TODO: You can also define nVertices as verticePos.size() - 1 (because the first pos is repeated)

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
    double closestVertex = 0;
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

// not being used
bool Polygon::edgeinMP(vec2r& edgePos, int& closestCorner, MaterialPoint& MP, vec2r& tang, vec2r& normal,
                       double& testdn) {
  int i, j = 0;
  bool c = false;
  double nbCorners = 4;

  for (i = 0, j = nbCorners - 1; i < nbCorners; j = i++) {
    if (((MP.corner[i].y > edgePos.y) != (MP.corner[j].y > edgePos.y)) &&
        (edgePos.x <
         (MP.corner[j].x - MP.corner[i].x) * (edgePos.y - MP.corner[i].y) / (MP.corner[j].y - MP.corner[i].y) +
             MP.corner[i].x)) {
      c = !c;
    }
  }

  if (c == true) {
    tang = MP.corner[2] - MP.corner[3];  // this has to be corrected
    tang.normalize();
    normal.x = tang.y;
    normal.y = -tang.x;

    // finding closest corner (to the edge that we're analyzing)
    // double closestCorner = 0;
    double distanceClosest = 2 * MP.size;  // random initial value that is big enough
    for (int r = 0; r < 4; r++) {
      double normDiff = norm(edgePos - MP.corner[r]);
      // std::cout<<normDiff<< " "<<distanceClosest<<std::endl;
      if (normDiff < distanceClosest) {
        distanceClosest = normDiff;
        closestCorner = r;
      }
    }
    // once found the closest corner, we calculate the testdn
    testdn = (edgePos - MP.corner[closestCorner]) * -normal;
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
