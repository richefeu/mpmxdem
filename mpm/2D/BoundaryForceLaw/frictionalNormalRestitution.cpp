// NOTE: Boundary that is applied by default in the Obstacle constructor

#include "IncrementalLinear.hpp"
#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>

#include <factory.hpp>
static Registrar<BoundaryType, IncrementalLinear> registrar("IncrementalLinear");

// vectorMP, vectorObstacles, dataTable, id_kn, id_kt, id_en2, id_mu, dt
void IncrementalLinear::calculateContactForces(std::vector<MaterialPoint>& MP, DataTable dataTable,
                                               Obstacle* currentObstacle, size_t id_kn, size_t id_kt, size_t id_en2,
                                               size_t id_mu, size_t, double dt) {
  double kn, kt, en2, mu;
  size_t g1, g2;
  for (size_t nn = 0; nn < currentObstacle->Neighbors.size(); ++nn) {
    size_t pn = currentObstacle->Neighbors[nn].PointNumber;
    double dn;
    int corner = currentObstacle->touch(MP[pn], dn);

    // if it returns -1 there is no contact, else it returns a positive number
    if (corner >= 0) {  // Check if there is contact
      g1 = (size_t)(MP[pn].groupNb);
      g2 = currentObstacle->group;
      kn = dataTable.get(id_kn, g1, g2);
      kt = dataTable.get(id_kt, g1, g2);
      en2 = dataTable.get(id_en2, g1, g2);
      mu = dataTable.get(id_mu, g1, g2);

      vec2r N, T;
      currentObstacle->getContactFrame(MP[pn], N, T);

      // === Normal force
      if (dn < 0.0) {  // overlapping
        double delta_dn = dn - currentObstacle->Neighbors[nn].dn;
        if (delta_dn > 0.0) {  // Unloading
          currentObstacle->Neighbors[nn].fn = -kn * en2 * dn;
        } else if (delta_dn < 0.0) {  // Loading
          currentObstacle->Neighbors[nn].fn += -kn * delta_dn;
        } else {
          currentObstacle->Neighbors[nn].fn = -kn * dn;  // First time loading
        }
      }

      currentObstacle->Neighbors[nn].dn = dn;

      // === Friction force
      vec2r lever = MP[pn].pos - currentObstacle->pos;
      vec2r zCrossLever(-lever.y, lever.x);
      double delta_dt =
          ((MP[pn].pos - MP[pn].prev_pos) - dt * (currentObstacle->vel + currentObstacle->vrot * zCrossLever)) * T;

      currentObstacle->Neighbors[nn].ft += -kt * delta_dt;
      double threshold = mu * currentObstacle->Neighbors[nn].fn;

      if (currentObstacle->Neighbors[nn].ft > threshold) currentObstacle->Neighbors[nn].ft = threshold;
      if (currentObstacle->Neighbors[nn].ft < -threshold) currentObstacle->Neighbors[nn].ft = -threshold;

      // === Resultant force
      vec2r f = currentObstacle->Neighbors[nn].fn * N + currentObstacle->Neighbors[nn].ft * T;
      MP[pn].contactf.x = f.x * -1;  // just to show the contact forces in the vtk
      MP[pn].contactf.y = f.y * -1;
      MP[pn].f += f;
      currentObstacle->force -= f;

      // === Resultant moment (only on the obstacle)
      currentObstacle->mom += cross(lever, -f);
    } else {
      currentObstacle->Neighbors[nn].dn = 0.0;
      currentObstacle->Neighbors[nn].dt = 0.0;
      currentObstacle->Neighbors[nn].fn = 0.0;
      currentObstacle->Neighbors[nn].ft = 0.0;
      MP[pn].contactf.reset();
    }  // end if else touch
  }    // end for-loop over neighbors nn
}
