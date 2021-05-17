#include "Line.hpp"
#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>

#include <factory.hpp>
static Registrar<Obstacle, Line> registrar("Line");
std::string Line::getRegistrationName() { return std::string("Line"); }

void Line::read(std::istream& is) {
  vec2r end;
  is >> group >> pos >> end;
  t = end - pos;
  len = t.normalize();
  n.x = t.y;
  n.y = -t.x;  // so that n ^ t = z

  std::string driveMode;
  is >> driveMode;
  if (driveMode == "freeze") {
    isFree = false;
    vel.reset();
  } else if (driveMode == "velocity") {
    isFree = false;
    is >> vel;
  } else {
    std::cerr << "@Line::read, driveMode " << driveMode << " is not allowed!" << std::endl;
  }
}

void Line::write(std::ostream& os) {
  os << group << ' ' << pos << ' ' << pos + t * len << ' ' << "velocity " << vel << '\n';
}

int Line::touch(MaterialPoint& MP, double& dn) {
  int Touch = -1;
  vec2r c = MP.pos - pos;
  double radiusMP = 0.5 * sqrt(MP.vol0);
  dn = c * n - radiusMP;
  if (dn < 0.0) {
    double proj = c * t;
    if (proj >= 0.0 && proj <= len) Touch = 1;
  }
  return Touch;
}

void Line::getContactFrame(MaterialPoint&, vec2r& N, vec2r& T) {
  // Remark: the line is not supposed to rotate
  N = n;
  T = t;
}

void Line::checkProximity(MPMbox& MPM) {
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
    double dstt = c * t;
    if (dstt > -sumSecurDist && dstt < len + sumSecurDist) {
      double dstn = c * n;
      if (dstn < sumSecurDist) {
        N.PointNumber = p;
        Neighbors.push_back(N);
      }
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

int Line::addVtkPoints(std::vector<vec2r>& coords) {
  coords.push_back(pos);
  coords.push_back(pos + len * t);
  return 2;
}
