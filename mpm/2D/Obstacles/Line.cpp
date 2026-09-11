#include "Line.hpp"
#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

std::string Line::getRegistrationName() { return std::string("Line"); }

void Line::read(std::istream& is) {
  vec2r end;
  is >> group >> pos >> end;
  udir = end - pos;
  len = udir.normalize();
  normal.x = udir.y;
  normal.y = -udir.x;  // so that n ^ t = z

  std::string driveMode;
  is >> driveMode;
  if (driveMode == "freeze") {
    drive_mode =FREEZE;
    vel.reset();
  } else if (driveMode == "velocity") {
    drive_mode =IMPOSE_VELOCITY;
    steps = 1;
    is >> vel;
  } else if (driveMode == "Steps") {
    drive_mode =IMPOSE_VELOCITY;
    is >> steps;
    for (int i = 0; i < steps ; i++) {
      double stepTime;
      vec2r impVel;
      is >> stepTime >> impVel;
      stepTimes.push_back(stepTime);
      impVels.push_back(impVel);
    }
    vel = impVels[0];
  } else if (driveMode == "force") {
    drive_mode =IMPOSE_FORCE;
    steps = 1;
    double impForce;
    double damp;
    is >> impForce >> mass >> damp;
    impForces.push_back(impForce);
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
  double proj = c * udir;
  vec2r N ;
  // Determine normal vector depending if the MP is within the line or at one of its ends
    if (proj >= 0.0 && proj <= len) { 
      N = normal;
    }
    else if (proj < 0){
      c.normalize();
      N = c;
    }
    else {
      c -= len*udir;
      c.normalize();
      N = c;
    }
  dn = c *N- radiusMP;
  if (dn < 0.0) {
      Touch = 1;
    }
  return Touch;
}

void Line::getContactFrame(MaterialPoint& MP, vec2r& N, vec2r& T) {
  // // Remark: the line is not supposed to rotate
  // N = normal;
  // T = udir;
  vec2r c = MP.pos - pos;
  double proj = c * udir;
  if (proj >= 0.0 && proj <= len) {
    N = normal;
    T = udir;
  }
  else if (proj < 0.0) {
    N = c;
    N.normalize();
    T.x = -N.y;
    T.y = N.x;
  }else if (proj > len) {
    c -= len*udir;
    N = c;
    N.normalize();
    T.x = -N.y;
    T.y = N.x;
  }
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
      double dstn = c * normal;
      if (dstn < sumSecurDist) {
        N.PointNumber = p;
        Neighbors.push_back(N);
      }
    }
  }

  // Get the known forces back in the vector 'Neighbors'
  size_t istore = 0;
  for (size_t inew = 0; inew < Neighbors.size(); inew++) {
    while (istore < Store.size() && Neighbors[inew].PointNumber < Store[istore].PointNumber) {
      ++istore;
    }
    if (istore == Store.size()) {
      break;
    }

    if (Store[istore].PointNumber == Neighbors[inew].PointNumber) {
      Neighbors[inew] = Store[istore];
      ++istore;
    }
  }
}

void Line::updateImposedVelocity(MPMbox& MPM) {
  for (int i = 0; i < steps ; i++) {
    if (MPM.t <= MPM.finalTime * stepTimes [i]){
      vel = impVels[i];
      break;
    }
  }
}

bool Line::inside(vec2r& x) {
  vec2r l = x - pos;
  return (l * normal < 0.0);
}
