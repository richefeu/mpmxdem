#include "Circle.hpp"
#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

#include <cmath>

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
    mass = M_PI * R * R * density;
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
    double density = mass / (M_PI * R * R);
    os << "free " << density << ' ' << vel << ' ' << vrot << '\n';
  }
}

int Circle::touch(MaterialPoint& MP, double& dn) {
  vec2r c = MP.pos - pos;
  double radiusMP = 0.5 * sqrt(MP.vol);
  dn = norm(c) - R - radiusMP;
  if (dn < 0.0) {
    return 1;
  } else {
    return -1;
  }
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

  // Get the contact history back
  restoreNeighborHistory(Store);
}

bool Circle::inside(vec2r& x) {
  vec2r l = x - pos;
  return (norm2(l) < R * R);
}
