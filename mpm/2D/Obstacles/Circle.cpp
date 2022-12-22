#include "Circle.hpp"
#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

//#include "factory.hpp"
//static Registrar<Obstacle, Circle> registrar("Circle");
std::string Circle::getRegistrationName() { return std::string("Circle"); }

void Circle::read(std::istream& is) {
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

void Circle::write(std::ostream& os) {
  os << group << ' ' << pos << ' ' << R << ' ';
  if (isFree == false) {
    os << "velocity " << vel << '\n';
  } else {
    double density = mass / (Mth::pi * R * R);
    os << "free " << density << ' ' << vel << ' ' << vrot << '\n';
  }
}

int Circle::touch(MaterialPoint& MP, double& dn) {
  vec2r c = MP.pos - pos;
  double radiusMP = 0.5 * sqrt(MP.vol);
  dn = norm(c) - R - radiusMP;
  if (dn < 0.0)
    return 1;  // true;
  else
    return -1;
}

void Circle::getContactFrame(MaterialPoint& MP, vec2r& N, vec2r& T) {
  vec2r N1 = MP.pos - pos;
  N = N1.normalized();
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

bool Circle::MPisInside(MaterialPoint& MP) {
  vec2r l = MP.pos - pos;
  double radiusMP = 0.5 * sqrt(MP.vol0);
  double sumR = (R + radiusMP);
  return (norm2(l) < sumR * sumR);
}
