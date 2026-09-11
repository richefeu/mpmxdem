#include "DPCPlotMean.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"
#include "ConstitutiveModels/ConstitutiveModel.hpp"

#include "fileTool.hpp"

void DPCPlotMean::read(std::istream& is) {
  is >> nrec;
  nstep = nrec;

  PQ_filename = box->result_folder + fileTool::separator() + "DPCPlotMeanPQ.txt";
  parameters_filename = box->result_folder + fileTool::separator() + "DPCPlotMeanParams.txt";

  std::cout << "DPCPlotMean : filenames are " << PQ_filename <<", " << parameters_filename << std::endl;
  
  PQ_file.open(PQ_filename.c_str());
  parameters_file.open(parameters_filename.c_str());

}


void DPCPlotMean::exec() {
  
  size_t nbMP = box->MP.size();
  if (0 == nbMP) {return;}

  double temp_meanP = 0;
  double temp_meanQ = 0;
  double temp_meanPb = 0;
  double temp_meanbeta = 0;
  double temp_meanR = 0;
  double temp_meand = 0;
  double temp_meanE = 0;
  double temp_meanNu = 0;
  double temp_meanRD = 0;
  
  for (size_t p = 0 ; p < nbMP ; p++) {

    MPStress = box->MP[p].stress;
    MPStrain = box->MP[p].strain;
    
    double diff_xx_yy = MPStress.xx-MPStress.yy;
    temp_meanP += -0.5*(MPStress.xx + MPStress.yy);
    temp_meanQ += sqrt(3*(0.25*diff_xx_yy*diff_xx_yy + MPStress.xy*MPStress.xy));

    if (box->MP[p].constitutiveModel->getRegistrationName() == "DruckerPragerCap") {
      std::vector<double> params = box->MP[p].constitutiveModel->getOtherParams(p);
      temp_meanPb += params[0];
      temp_meanR += params[1];
      temp_meanbeta+= params[2];
      temp_meand += params[3];
      temp_meanE += params[4];
      temp_meanNu += params[5];
      temp_meanRD += params[6];
    }
    else {
      std::cout<<"Error : MP n°"<< p << " does not follow the DPC constitutive model" << std::endl;
    }
  }
  meanP = temp_meanP/nbMP ;
  meanQ = temp_meanQ/nbMP ;
  meanPb = temp_meanPb/nbMP ;
  meanR = temp_meanR/nbMP ;
  meanbeta = temp_meanbeta/nbMP ;
  meand = temp_meand/nbMP ;
  meanE = temp_meanE/nbMP ;
  meanNu = temp_meanNu/nbMP ;
  meanRD = temp_meanRD/nbMP ;
}

void DPCPlotMean::record() {
  if (start) {
    PQ_file << "#t(s)\t P(N/m²)\t Q(N/m²)" << std::endl; 
    parameters_file << "#t(s)\t Pb(N/m²)\t R(1)\t beta(rad)\t d(N/m²)\t E(N/m²)\t ν(1)\t RD" << std::endl;
    start = false;
  }

  PQ_file << std::scientific << std::setprecision(std::numeric_limits<double>::digits10 + 1);
  PQ_file << box->t << ' ' << meanP << ' ' << meanQ << std::endl; 
  parameters_file << std::scientific << std::setprecision(std::numeric_limits<double>::digits10 + 1);
  parameters_file << box->t << " " << meanPb << " " << meanR << " " << meanbeta << " " 
                  << meand << " " << meanE << " " << meanNu << " " << meanRD << std::endl;
}

void DPCPlotMean::end() { 
  PQ_file.close(); 
  parameters_file.close();
}

