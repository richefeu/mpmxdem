#include "DPPlotSingle.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

#include "fileTool.hpp"

void DPPlotSingle::read(std::istream& is) {
  std::string Filename;
  is >> nrec >> Filename >> MP_id;
  nstep = nrec;

  filename = box->result_folder + fileTool::separator() + Filename;
  std::cout << "DPPlotSingle: filename is " << filename << std::endl;
  std::cout << "DPPlotSingle: Tracked MP initial coordinates : " << box->MP[MP_id].pos.x << ", " << box->MP[MP_id].pos.y << std::endl;
  file.open(filename.c_str());
  is >> FrictionAngle >> Cohesion;
  double sinFrictionAngle = sin(FrictionAngle);
  double cosFrictionAngle = cos(FrictionAngle);
  alpha = 2*sinFrictionAngle/(sqrt(3)*(3-sinFrictionAngle));
	k = 6*Cohesion*cosFrictionAngle/(sqrt(3)*(3-sinFrictionAngle));
}

void DPPlotSingle::exec() {
  MPStress.reset();
  MPStrain.reset();
  size_t nbMP = box->MP.size();
  if (0 == nbMP) {return;}
 
  MPStress = box->MP[MP_id].stress;
  MPStrain = box->MP[MP_id].strain;

  double diff_xx_yy = MPStress.xx-MPStress.yy;
  
  P = 0.5*(MPStress.xx + MPStress.yy);
  Q = sqrt(1.5*(0.25*diff_xx_yy*diff_xx_yy + MPStress.xy*MPStress.xy));
  yieldQ = sqrt(3)*(k - alpha*2*P);
  // double yieldF = 2*alpha*P + Q/sqrt(3) - k;
  // std::cout<<"Spy : "<<yieldF<<std::endl;
}

void DPPlotSingle::record() {
  file << std::scientific << std::setprecision(std::numeric_limits<double>::digits10 + 1);
  file << box->t << ' ' << P << ' ' << Q << " " << yieldQ << std::endl;
}

void DPPlotSingle::end() { file.close(); }

