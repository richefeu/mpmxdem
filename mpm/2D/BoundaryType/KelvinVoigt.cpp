// NOTE: This is not incremental
// This is the spring-dashpot model

#include "KelvinVoigt.hpp"
#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>

#include <factory.hpp>
static Registrar<BoundaryType, KelvinVoigt> registrar("KelvinVoigt");

// vectorMP, vectorObstacles, dataTable, id_kn, id_kt, id_en2, id_mu, dt
void KelvinVoigt::calculateContactForces(std::vector<MaterialPoint>& MP, DataTable dataTable, Obstacle* currentObstacle,
                                         size_t id_kn, size_t id_kt, size_t id_en2, size_t id_mu, size_t id_viscosity,
                                         double dt) {

  double kn, kt, en2, mu, viscosity;
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
      viscosity = dataTable.get(id_viscosity, g1, g2);

      vec2r N, T;
      currentObstacle->getContactFrame(MP[pn], N, T);

      // === Normal force
      if (dn < 0.0) {  // overlapping
        // in all cases
        // TODO: take into account the velocity of the obstacle (velobst - velmp or viceversa)
        double normalVel = MP[pn].vel * N;
        // currentObstacle->Neighbors[nn].fn = -kn * dn + (-viscosity) * dn/dt;
        double visc = 2 * sqrt(MP[pn].mass * kn);
        // std::cout << "visc: "<<visc << '\n';
        // currentObstacle->Neighbors[nn].fn = -kn * dn - viscosity * normalVel;
        currentObstacle->Neighbors[nn].fn = -kn * dn - visc * normalVel;
        // std::cout << "regular: "<< -kn * dn <<"  dashpot: "<< currentObstacle->Neighbors[nn].fn<< '\n';
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
      // std::cout<<f<<std::endl;
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
