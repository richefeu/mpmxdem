#include "SigmaN_vs_fN.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"
#include "Obstacles/Obstacle.hpp"

#include "fileTool.hpp"

void SigmaN_vs_fN::read(std::istream& is) {
  std::string Filename;
  is >> nrec >> Filename >> MP_id;
  nstep = nrec;

  filename = box->result_folder + fileTool::separator() + Filename;
  std::cout << "SigmaN_vs_fN: filename is " << filename << std::endl;
  std::cout << "SigmaN_vs_fN: Tracked MP initial coordinates : " << box->MP[MP_id].pos.x << ", " << box->MP[MP_id].pos.y << std::endl;
  file.open(filename.c_str());
}

void SigmaN_vs_fN::exec() {
  
  for (size_t o = 0; o < box->Obstacles.size(); ++o){
    for (size_t nn = 0; nn < box->Obstacles[o]->Neighbors.size(); ++nn) {
      if (nn == MP_id) {
        if (box->Obstacles[o]->Neighbors[nn].dn >= 0.0) continue;
        f = box->MP[MP_id].f;
        sigma_n = box->MP[MP_id].stress.yy;
        l = sqrt(box->MP[MP_id].corner[1].x*box->MP[MP_id].corner[0].x + box->MP[MP_id].corner[1].y*box->MP[MP_id].corner[0].y);
      }
    }
  }
}
void SigmaN_vs_fN::record() {
  file << std::scientific << std::setprecision(std::numeric_limits<double>::digits10 + 1);
  file << box->t << ' ' << sigma_n << ' ' << f.y << std::endl;
}

void SigmaN_vs_fN::end() { file.close(); }

