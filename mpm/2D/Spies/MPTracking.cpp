#include "MPTracking.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

#include "fileTool.hpp"

MPTracking::MPTracking() {
  MP_Selector.actionForId = [this](MPMbox* /*B*/, size_t p) { this->MP_ids.push_back(p); };

  MP_Selector.ContainerSize = [](MPMbox* B) -> size_t { return B->MP.size(); };

  MP_Selector.getXY = [](MPMbox* B, size_t p, double& x, double& y) {
    x = B->MP[p].pos.x;
    y = B->MP[p].pos.y;
  };
}

void MPTracking::read(std::istream& is) {
  std::string Filename;
  is >> nrec >> Filename >> MP_Selector;	
  nstep = nrec;

  filename = box->result_folder + fileTool::separator() + Filename;
  std::cout << "MPTracking: filename is " << filename << std::endl;
  file.open(filename.c_str());
	MP_Selector.execute(box);	
}

void MPTracking::exec() {
  meanStress.reset();
	meanStrain.reset();
	size_t nbMP = box->MP.size();
	if (0 == nbMP) return;
	
	size_t p = 0;
  for (size_t id = 0; id < MP_ids.size(); id++) {
		p = MP_ids[id];
    meanStress += box->MP[p].stress;
		meanStrain += box->MP[p].strain;
  }
	meanStress *= (1.0 / (double)MP_ids.size());
	meanStrain *= (1.0 / (double)MP_ids.size());
}

void MPTracking::record() {
	file << std::scientific << std::setprecision(std::numeric_limits<double>::digits10 + 1);
  file << box->t << ' ' << meanStress << ' ' << meanStrain << std::endl;
}

void MPTracking::end() {
	file.close();
}