#include "LeftBiplanar.hpp"
#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>

#include <factory.hpp>
static Registrar<Obstacle, LeftBiplanar> registrar("LeftBiplanar");

using namespace std;
void LeftBiplanar::read(std::istream& is) {
  is >> from1 >> l1 >> l2 >> R >> angle;
  to1.x = from1.x - l1;
  to1.y = from1.y;

  center.x = to1.x;
  center.y = to1.y + R;
  vec2r u = to1 - center;
  angle = angle * Mth::deg2rad;  // converting back to radians
  anglep0 = atan2(u.y, u.x);
  from2.x = center.x + R * cos(anglep0 - angle);
  from2.y = center.y + R * sin(anglep0 - angle);
  to2.x = from2.x - l2 * cos(angle);
  to2.y = from2.y + l2 * sin(angle);

  // normal and tangent vectors
  t1 = to1 - from1;
  t1.normalize();
  n1.x = t1.y;
  n1.y = -t1.x;
  t2 = to2 - from2;
  t2.normalize();
  n2.x = t2.y;
  n2.y = -t2.x;

  // For compatibility with the new version
  group = 0;
  isFree = false;
}

int LeftBiplanar::touch(MaterialPoint& MP, double& dn) {
  contactWith = 0;
  vec2r c1 = MP.pos - from1;
  vec2r c2 = MP.pos - from2;
  cArc = MP.pos - center;

  double len = sqrt(MP.vol) / 2.0;  // because vol is size^2
  double dst1 = c1 * n1 - len;
  double dst2 = c2 * n2 - len;
  double angleMP = atan2(cArc.y, cArc.x);
  double dstArc = R - norm(cArc) - len;
  cArc.normalize();
  nArc = -cArc;
  tArc.x = -nArc.y;
  tArc.y = nArc.x;

  if (dstArc < 0.0 and (angleMP < anglep0 and angleMP > (anglep0 - angle))) {  // for arc
    contactWith = 3;
    dn = dstArc;
    return 0;
  }

  else if (dst1 < 0.0) {  // for horizontal line
    dn = dst1;
    contactWith = 1;
    return 0;
  }

  else if (dst2 < 0.0) {  // for angled line
    dn = dst2;
    contactWith = 2;
    return 0;
  }

  else
    return -1;
}

void LeftBiplanar::getContactFrame(MaterialPoint&, vec2r& N, vec2r& T) {
  if (contactWith == 0) std::cerr << "@LeftBiplanar::getContactFrame, Problem with contact" << std::endl;

  if (contactWith == 1) {
    N = n1;
    T = t1;
  }
  if (contactWith == 2) {
    N = n2;
    T = t2;
  }
  if (contactWith == 3) {
    N = nArc;
    T = tArc;
  }
}

void LeftBiplanar::checkProximity(MPMbox& MPM)  // part for workSlice added here. To be moved somewhere else
{
  // Temporarily store the forces
  std::vector<Neighbor> Store = Neighbors;

  // Rebuild the list
  Neighbors.clear();
  Neighbor N;

  for (size_t p = 0; p < MPM.MP.size(); p++) {
    N.PointNumber = p;
    Neighbors.push_back(N);
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

int LeftBiplanar::addVtkPoints(std::vector<vec2r>& coords) {
  double nbPoints = 0;
  coords.push_back(from1);
  nbPoints += 1;

  const int nbSectors = 50;

  vec2r P;
  double inc = angle / (double)nbSectors;
  for (int i = 0; i <= nbSectors; i++) {
    P.x = center.x + R * cos(-i * inc + anglep0);
    P.y = center.y + R * sin(-i * inc + anglep0);
    coords.push_back(P);
  }
  nbPoints += nbSectors + 1;

  coords.push_back(to2);
  nbPoints += 1;

  return nbPoints;
}
