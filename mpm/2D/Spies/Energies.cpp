#include "Energies.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"
#include "Obstacles/Obstacle.hpp"

#include "fileTool.hpp"

void Energies::read(std::istream& is) {
  std::string Filename;
  is >> nrec >> Filename;
  nstep = 1;  // exec is called at each time step

  filename = box->result_folder + fileTool::separator() + Filename;
  std::cout << "Energies: filename is " << filename << std::endl;
  file.open(filename.c_str());
}

void Energies::exec() {
  Ec = 0;
  Ep = 0;
  Eel = 0;
  Epl = 0;

  for (size_t p = 0; p < box->MP.size(); p++) {
    double m = box->MP[p].mass ;
    double vx = box->MP[p].vel[0];
    double vy = box->MP[p].vel[1];
    Ec += 0.5 * m * (vx*vx + vy*vy);

    double g = -box->gravity[1];
    double h = box->MP[p].pos[1];
    Ep += m*g*h;

    double V = box->MP[p].vol;
    mat4r sigma = box->MP[p].stress;
    mat4r eps = box->MP[p].strain;
    Eel += V*(sigma.xx*eps.xx + sigma.yy*eps.yy + 2*sigma.xy*eps.xy);
  };
}
//   // MP Works due to forces with obstacles
//   for (size_t o = 0; o < box->Obstacles.size(); ++o) {
//     for (size_t nn = 0; nn < box->Obstacles[o]->Neighbors.size(); ++nn) {

//       if (box->Obstacles[o]->Neighbors[nn].dn >= 0.0) continue;
//       size_t pn = box->Obstacles[o]->Neighbors[nn].PointNumber;

//       vec2r disp = box->MP[pn].pos - box->MP[pn].prev_pos;  // here the obstacle is not supposed to move
//       vec2r N, T;
//       box->Obstacles[o]->getContactFrame(box->MP[pn], N, T);
//       double delta_dn = disp * N;
//       double delta_dt = disp * T;

//       MP_Wn = delta_dn * box->Obstacles[o]->Neighbors[nn].fn;
//       Wn_tot += MP_Wn;

//       MP_Wt = delta_dt * box->Obstacles[o]->Neighbors[nn].ft;
//       Wt_tot += MP_Wt;
//     }
//   }
// 

void Energies::record() {
  file << box->t << " " << Ec << " " << Ep << " " << Eel << " " << Epl << std::endl;
}

void Energies::end(){
  file.close();
}