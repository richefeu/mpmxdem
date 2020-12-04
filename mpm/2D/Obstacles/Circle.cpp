#include "Circle.hpp"
#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>

#include <factory.hpp>
static Registrar<Obstacle, Circle> registrar("Circle");

// TODO: It should have the the contact as the square. Double check, not only if corners are inside the circle (it might
// change some of the results)
void Circle::read(std::istream& is) {
  // std::string boundaryName;
  is >> group >> pos >> R;
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
    mass = Mth::pi * R * R * density;
    I = 0.5 * mass * R * R;
    is >> vel >> vrot;
  } else {
    std::cerr << "@Circle::read, driveMode " << driveMode << " is unknown!" << std::endl;
  }
}

int Circle::touch(MaterialPoint& MP, double& dn) {
  /* //part for corners. removed (21/03/2018)
  int Corner = -1;
  vec2r c;
  // Corner 0
  c = MP.corner[0] - pos;
  dn = norm(c) - R;
  if (dn < 0.0) Corner = 0;

  //there is (was) a bug here but apparently its not affecting the simulations. if e.g. dn = 3 and then dst < dn but
  still positive (so no interpenetration) then youre assigning a corner anyways, which you shouldnt.
  // Corners 1, 2 and 3
  for (int r = 1 ; r < 4 ; r++) {
          c = MP.corner[r] - pos;
          double dst = norm(c) - R;
          //~ if (dst < dn) {  //original line
          if (dst < dn and dst < 0.0) {
                  Corner = r;
                  dn = dst;
          }
  }
  return Corner;
  */
  int Corner = -1;
  vec2r c = MP.pos - pos;
  double len = sqrt(MP.vol) / 2.0;
  dn = norm(c) - R - len;
  if (dn < 0.0)
    return 1;  // true;
  else
    return Corner;
  // else return false;
}

void Circle::getContactFrame(MaterialPoint& MP, vec2r& N, vec2r& T) {
  vec2r N1 = MP.pos - pos;
  N = N1.normalized();  // it was normalize before (27/03/2018)
  T.x = -N.y;
  T.y = N.x;
}

void Circle::checkProximity(MPMbox& MPM) {
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
    double dst = norm(c) - R;
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

int Circle::addVtkPoints(std::vector<vec2r>& coords) {
  const int nbSectors = 100;
  vec2r P;
  double inc = Mth::_2pi / (double)nbSectors;
  coords.push_back(pos);
  for (int i = 0; i <= nbSectors; i++) {
    P.x = pos.x + R * cos(rot + i * inc);
    P.y = pos.y + R * sin(rot + i * inc);
    coords.push_back(P);
  }
  return nbSectors + 1 + 1;  // one of the 1 is because of the central pos
}

bool Circle::MPisInside(MaterialPoint& MP) {
  vec2r l = MP.pos - pos;
  double radiusMP = 0.5 * MP.vol;
  double sumR = (R + radiusMP);
  return (norm2(l) < sumR * sumR);
}
