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
  file.open(filename.c_str());
}

void MeanStress::exec() {
  meanStress.reset();
  meanStrain.reset();
	size_t nbMP = box->MP.size();
	if (0 == nbMP) return;
	
  for (size_t p = 0; p < nbMP; p++) {
    meanStress += box->MP[p].stress;
    meanStrain += box->MP[p].strain;
  }
	meanStress *= (1.0 / (double)nbMP);
	meanStrain *= (1.0 / (double)nbMP);
}

void MeanStress::record() {
  if (start){
    file << "#time(s)\t Mean Sigmaxx\t Mean Sigmaxy\t Mean Sigmayx\t  Mean Sigmayy\t Mean Epsxx\t Mean Epsxy\t Mean Epsyx\t Mean Epsyy\t" << std::endl;
    start = false;
  }
	file << std::scientific << std::setprecision(std::numeric_limits<double>::digits10 + 1);
  file << box->t << ' ' << meanStress << ' ' << meanStrain << std::endl;
}

void MeanStress::end() {
	file.close();
}