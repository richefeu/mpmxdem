#include "move_MP.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

//#include "factory.hpp"
//static Registrar<Command, move_MP> registrar("move_MP");

void move_MP::read(std::istream& is) { is >> groupNb >> x0 >> y0 >> dx >> dy >> thetaDeg; }

void move_MP::exec() {
  double theta = thetaDeg * Mth::deg2rad;
  mat4r rotation;
  rotation.xx = cos(theta);
  rotation.xy = -sin(theta);
  rotation.yx = sin(theta);
  rotation.yy = cos(theta);

  // FIXME: I DON'T UNDERSTAND WHY WE DO THAT ????????
  /*
  for (size_t p = 0; p < box->MP.size(); p++) {
    if (box->MP[p].groupNb == groupNb) {
      box->MP[p].corner[0].x = -0.5 * box->MP[p].size;
      box->MP[p].corner[0].y = -0.5 * box->MP[p].size;
      box->MP[p].corner[1].x = +0.5 * box->MP[p].size;
      box->MP[p].corner[1].y = -0.5 * box->MP[p].size;
      box->MP[p].corner[2].x = +0.5 * box->MP[p].size;
      box->MP[p].corner[2].y = +0.5 * box->MP[p].size;
      box->MP[p].corner[3].x = -0.5 * box->MP[p].size;
      box->MP[p].corner[3].y = +0.5 * box->MP[p].size;
    }
  }
  */
  
  double newx, newy;
  for (size_t p = 0; p < box->MP.size(); p++) {
    if (box->MP[p].groupNb == groupNb) {

      // box->MP[p].F = rotation * box->MP[p].F * rotation.transpose();
      box->MP[p].F = rotation;  // initially we can say that
      newx = x0 + (box->MP[p].pos.x - x0) * rotation.xx + (box->MP[p].pos.y - y0) * rotation.xy + dx;
      newy = y0 + (box->MP[p].pos.x - x0) * rotation.yx + (box->MP[p].pos.y - y0) * rotation.yy + dy;
      box->MP[p].pos.x = newx;
      box->MP[p].pos.y = newy;

      // box->MP[p].updateCornersFromF();
      // TODO: to be improved (look into box->MP[p].updateCornersFromF(); to see if you can use it)
      for (int c = 0; c < 4; c++) {

        newx = box->MP[p].pos.x + (box->MP[p].corner[c].x) * rotation.xx + (box->MP[p].corner[c].y) * rotation.xy;
        newy = box->MP[p].pos.y + (box->MP[p].corner[c].x) * rotation.yx + (box->MP[p].corner[c].y) * rotation.yy;
        box->MP[p].corner[c].x = newx;
        box->MP[p].corner[c].y = newy;

        // or this one line should work
        // box->	MP[p].corner[c] = box->MP[p].pos + box->MP[p].F*box->	MP[p].corner[c];
      }
    }
  }
}
