#include "EnergyBalance.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"
#include "Obstacles/Obstacle.hpp"

#include "fileTool.hpp"


// EN COURS DE DEV !!!!

void EnergyBalance::read(std::istream& is) {
  std::string Filename;
  is >> nrec >> Filename;
  nstep = 1;  // exec is called at each time step

  filename = box->result_folder + fileTool::separator() + Filename;
  std::cout << "Work: filename is " << filename << std::endl;
  file.open(filename.c_str());
}

void EnergyBalance::exec() {
  double MP_Wn;
  double MP_Wt;
  double MP_Wint;
  double MP_Wp;

  for (size_t p = 0; p < box->MP.size(); p++) {
    // Internal work
    MP_Wint = box->MP[p].vol *
              (box->MP[p].stress.xx * box->MP[p].deltaStrain.xx + box->MP[p].stress.yy * box->MP[p].deltaStrain.yy +
               2.0 * box->MP[p].stress.xy * box->MP[p].deltaStrain.xy);
    Wint_tot += MP_Wint;

    // Weight
    MP_Wp = box->MP[p].mass * box->gravity.y * (box->MP[p].pos - box->MP[p].prev_pos) * vec2r::unit_y();
    Wp_tot += MP_Wp;
  }

  // MP Works due to forces with obstacles
  for (size_t o = 0; o < box->Obstacles.size(); ++o) {
    for (size_t nn = 0; nn < box->Obstacles[o]->Neighbors.size(); ++nn) {

      if (box->Obstacles[o]->Neighbors[nn].dn >= 0.0) continue;
      size_t pn = box->Obstacles[o]->Neighbors[nn].PointNumber;

      vec2r disp = box->MP[pn].pos - box->MP[pn].prev_pos;  // here the obstacle is not supposed to move
      vec2r N, T;
      box->Obstacles[o]->getContactFrame(box->MP[pn], N, T);
      double delta_dn = disp * N;
      double delta_dt = disp * T;

      MP_Wn = delta_dn * box->Obstacles[o]->Neighbors[nn].fn;
      Wn_tot += MP_Wn;

      MP_Wt = delta_dt * box->Obstacles[o]->Neighbors[nn].ft;
      Wt_tot += MP_Wt;
    }
  }
}

void EnergyBalance::record() {
  file << box->t << " " << -Wn_tot << " " << -Wt_tot << " " << Wint_tot << " " << -Wn_tot - Wt_tot + Wint_tot << " "
       << Wp_tot << std::endl;
}

void EnergyBalance::end() {
  // fileSlices.close();
  // file.close();
}