#include "MeanStress.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

#include "fileTool.hpp"

void MeanStress::read(std::istream& is) {
  std::string Filename;
  is >> nrec >> Filename;
  nstep = nrec;

  filename = box->result_folder + fileTool::separator() + Filename;
  std::cout << "MeanStress: filename is " << filename << std::endl;
  // In visualisation mode -- see, cut -- MPMbox::read runs this very function on
  // the conf-file, and opening the output file in write mode TRUNCATES it. The
  // results of a computation must survive being looked at.
  if (box->computationMode == true) { file.open(filename.c_str()); }
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
  if (file.is_open() == false) { return; }
	file << std::scientific << std::setprecision(std::numeric_limits<double>::digits10 + 1);
  file << box->t << ' ' << meanStress << std::endl;
}

void MeanStress::end() {
  if (file.is_open()) { file.close(); }
}