#include "Line.hpp"
#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

std::string Line::getRegistrationName() { return std::string("Line"); }

void Line::read(std::istream& is) {
  vec2r end;
  is >> group >> pos >> end;
  udir = end - pos;
  len = udir.normalize();
  n.x = udir.y;
  n.y = -udir.x;  // so that n ^ t = z

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
  os << group << ' ' << pos << ' ' << pos + udir * len << ' ' << "velocity " << vel << '\n';
}

int Line::touch(MaterialPoint& MP, double& dn) {
  int Touch = -1;
  vec2r c = MP.pos - pos;
  double radiusMP = 0.5 * MP.size;
  dn = c * n - radiusMP;
  if (dn < 0.0) {
    double proj = c * udir;
    if (proj >= 0.0 && proj <= len) {
      Touch = 1;
    }
  }
  return Touch;
}

void Line::getContactFrame(MaterialPoint&, vec2r& N, vec2r& T) {
  // Remark: the line is not supposed to rotate
  N = n;
  T = udir;
}

void Line::checkProximity(MPMbox& MPM) {
  // Temporarily store the forces
  std::vector<Neighbor> Store = Neighbors;

  // Rebuild the list
  Neighbors.clear();
  Neighbor N;
  vec2r c;
  for (size_t p = 0; p < MPM.MP.size(); p++) {
    double sumSecurDistMin = 2 * MPM.MP[p].size;
    double sumSecurDist = MPM.MP[p].securDist + securDist;
    if (sumSecurDist < sumSecurDistMin) {
      sumSecurDist = sumSecurDistMin;
    }
    c = MPM.MP[p].pos - pos;
    double dstt = c * udir;
    if (dstt > -sumSecurDist && dstt < len + sumSecurDist) {
      double dstn = c * n;
      if (dstn < sumSecurDist) {
        N.PointNumber = p;
        Neighbors.push_back(N);
      }
    }
  }

  // Get the contact history back in the vector 'Neighbors'
  restoreNeighborHistory(Store);
}

bool Line::inside(vec2r& x) {
  vec2r l = x - pos;
  return (l * n < 0.0);
}
