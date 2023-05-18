#include "MPTracking.hpp"

#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>

#include <fileTool.hpp>

void MPTracking::read(std::istream& is) {
  /*
	selector.actionForNone = [](MPMbox *B) { };
  selector.actionForId = [](MPMbox *B, size_t p) {
    std::cout << B->element[p].pos << '\n';
  };
  selector.ContainerSize = [](AnyBox *B) -> size_t { return B->element.size(); };
  selector.getXY = [](AnyBox *B, size_t p, double &x, double &y) {
    x = B->element[p].pos.x;
    y = B->element[p].pos.y;
  };
	*/
	
  std::string Filename;
  is >> nrec >> Filename;// >> selector;	
  nstep = nrec;

  filename = box->result_folder + fileTool::separator() + Filename;
  std::cout << "MPTracking: filename is " << filename << std::endl;
  file.open(filename.c_str());
	
	
  for (size_t p = 0; p < box->MP.size(); p++) {
    if (box->MP[p].pos.x > 0.225 && box->MP[p].pos.x < 0.575 && box->MP[p].pos.y > 0.225 && box->MP[p].pos.y < 0.375) {
      MP_ids.push_back(p);
    }
  }
	
	
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