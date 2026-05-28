#include "MeanStressPQ_2D.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"
#include "ConstitutiveModels/ConstitutiveModel.hpp"
#include "ConstitutiveModels/DruckerPrager.hpp"

#include "fileTool.hpp"

void MeanStressPQ_2D::read(std::istream& is) {
  std::string Filename;
  is >> nrec >> Filename;
  nstep = nrec;

  filename = box->result_folder + fileTool::separator() + Filename;
  std::cout << "MeanStressPQ_2D: filename is " << filename << std::endl;
  file.open(filename.c_str());
}

void MeanStressPQ_2D::exec() {

  meanStress.reset();
	size_t nbMP = box->MP.size();
	if (0 == nbMP) return;
	
  for (size_t p = 0; p < nbMP; p++) {
    meanStress += box->MP[p].stress;
  }
	meanStress *= (1.0 / (double)nbMP);
  meanP = 0.5*(meanStress.xx + meanStress.yy);
  double diff_xx_yy = meanStress.xx - meanStress.yy;
  meanQ = sqrt(1.5*(0.25*diff_xx_yy*diff_xx_yy + 2*meanStress.xy*meanStress.xy));

  double alpha = 0.23094;
  double k = 1.2;
  yieldQ = sqrt(3)*(k - alpha*2*meanP);
}


void MeanStressPQ_2D::record() {
	file << std::scientific << std::setprecision(std::numeric_limits<double>::digits10 + 1);
  file << box->t << ' ' << meanStress << ' ' << meanP << ' ' << meanQ << ' ' << yieldQ << std::endl;
}

void MeanStressPQ_2D::end() {
	file.close();
}