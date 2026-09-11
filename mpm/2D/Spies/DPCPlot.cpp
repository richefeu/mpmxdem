#include "DPCPlot.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"
#include "ConstitutiveModels/ConstitutiveModel.hpp"

#include "fileTool.hpp"

void DPCPlot::read(std::istream& is) {
  is >> nrec >> nMP;
  nstep = nrec;

  Pvals.resize(nMP);
  Qvals.resize(nMP);
  Pbvals.resize(nMP);
  betavals.resize(nMP);
  Rvals.resize(nMP);
  dvals.resize(nMP);
  Evals.resize(nMP);
  Nuvals.resize(nMP);
  RDvals.resize(nMP);

  PQ_files.resize(nMP);
  PQ_filenames.resize(nMP);
  parameters_files.resize(nMP);
  parameters_filenames.resize(nMP);
  
  for (int i=0 ; i<nMP ; i++){
    size_t MP_id;
    is >> MP_id;
    tracked_MPs.push_back(MP_id);

    std::string PQ_filename = box->result_folder + fileTool::separator() + "DPCPlotPQ_MP" + std::to_string(MP_id) + ".txt";
    std::string parameters_filename = box->result_folder + fileTool::separator() + "DPCPlotParams_MP" + std::to_string(MP_id) + ".txt";

    PQ_filenames.push_back(PQ_filename);
    parameters_filenames.push_back(PQ_filename);

    std::cout << "DPCPlot, MP n°"<< MP_id << ": filenames are " << PQ_filename <<", " << parameters_filename << std::endl;
    std::cout << "DPCPlot, MP n°"<< MP_id << ": initial coordinates are " << box->MP[MP_id].pos.x << ", " << box->MP[MP_id].pos.y << std::endl;
    
    PQ_files[i] = new std::ofstream(PQ_filename.c_str());
    parameters_files[i] = new std::ofstream(parameters_filename.c_str());

  }
}

void DPCPlot::exec() {
  
  size_t nbMP = box->MP.size();
  if (0 == nbMP) {return;}

  for (int i = 0 ; i < nMP ; i++) {
    size_t MP_id = tracked_MPs[i];

    MPStress = box->MP[MP_id].stress;
    MPStrain = box->MP[MP_id].strain;
    
    double diff_xx_yy = MPStress.xx-MPStress.yy;
    Pvals[i] = -0.5*(MPStress.xx + MPStress.yy);
    Qvals[i] = sqrt(3*(0.25*diff_xx_yy*diff_xx_yy + MPStress.xy*MPStress.xy));

    if (box->MP[MP_id].constitutiveModel->getRegistrationName() == "DruckerPragerCap") {
      std::vector<double> params = box->MP[MP_id].constitutiveModel->getOtherParams(MP_id);
      Pbvals[i] = params[0];
      Rvals[i] = params[1];
      betavals[i] = params[2];
      dvals[i] = params[3];
      Evals[i] = params[4];
      Nuvals[i] = params[5];
      RDvals[i] = params[6];
    }
    else {
      std::cout<<"Error : MP n°"<< MP_id << " does not follow the DPC constitutive model" << std::endl;
    }
  }
}

void DPCPlot::record() {
  if (start) {
    for (int i = 0 ; i < nMP ; i++) {
      (*PQ_files[i]) << "#t(s)\t P(N/m²)\t Q(N/m²)" << std::endl; 
      (*parameters_files[i]) << "#t(s)\t Pb(N/m²)\t R(1)\t beta(rad)\t d(N/m²)\t E(N/m²)\t ν(1)\t RD" << std::endl;
    }
    start = false;
  }

  for (int i = 0 ; i < nMP ; i++) {
    (*PQ_files[i]) << std::scientific << std::setprecision(std::numeric_limits<double>::digits10 + 1);
    (*PQ_files[i]) << box->t << ' ' << Pvals[i] << ' ' << Qvals[i] << std::endl; 

    (*parameters_files[i]) << std::scientific << std::setprecision(std::numeric_limits<double>::digits10 + 1);
    (*parameters_files[i]) << box->t << " "  << Pbvals[i] << " " << Rvals[i] << " " << betavals[i] << " " << dvals[i] 
                          << " " << Evals[i] << " " << Nuvals[i] << " " << RDvals[i]<< std::endl;
    
  }
}

void DPCPlot::end() { 
  for (int i = 0 ; i < nMP ; i++) {
    PQ_files[i]->close(); 
    parameters_files[i]->close();
  }
}

