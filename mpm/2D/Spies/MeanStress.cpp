#include "MeanStress.hpp"

#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>

#include <fileTool.hpp>

void MeanStress::read(std::istream& is) {
  std::string Filename;
  is >> nrec >> Filename;
  nstep = nrec;

  filename = box->result_folder + fileTool::separator() + Filename;
  std::cout << "MeanStress: filename is " << filename << std::endl;
  file.open(filename.c_str());
}

void MeanStress::exec() {
  meanStress.reset();
	size_t nbMP = box->MP.size();
	if (0 == nbMP) return;
	
  for (size_t p = 0; p < nbMP; p++) {
    meanStress += box->MP[p].stress;
  }
	meanStress *= (1.0 / (double)nbMP);
}

void MeanStress::record() {
  file << box->t << ' ' << meanStress << std::endl;
}

void MeanStress::end() {
	file.close();
}