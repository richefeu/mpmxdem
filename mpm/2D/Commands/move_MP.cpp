#include "move_MP.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

void move_MP::read(std::istream& is) { is >> groupNb >> x0 >> y0 >> dx >> dy >> thetaDeg; }

void move_MP::exec() {
  double theta = thetaDeg * Mth::deg2rad;
  mat4r rotation;
  rotation.xx = cos(theta);
  rotation.xy = -sin(theta);
  rotation.yx = sin(theta);
  rotation.yy = cos(theta);

  
  double newx, newy;
  for (size_t p = 0; p < box->MP.size(); p++) {
    if (box->MP[p].groupNb == groupNb) {

      // box->MP[p].F = rotation * box->MP[p].F * rotation.transpose();
      box->MP[p].F = rotation;  // initially we can say that
      newx = x0 + (box->MP[p].pos.x - x0) * rotation.xx + (box->MP[p].pos.y - y0) * rotation.xy + dx;
      newy = y0 + (box->MP[p].pos.x - x0) * rotation.yx + (box->MP[p].pos.y - y0) * rotation.yy + dy;
      box->MP[p].pos.x = newx;
      box->MP[p].pos.y = newy;

      // Les coins n'existent plus dans MaterialPoint : ils etaient recalcules
      // depuis F a chaque pas, et la rotation qui se trouvait ici etait fausse
      // (defaut D7). F porte desormais seul l'orientation du point.
    }
  }
}
