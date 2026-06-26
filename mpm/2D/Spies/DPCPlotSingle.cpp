#include "DPCPlotSingle.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

#include "fileTool.hpp"

void DPCPlotSingle::read(std::istream& is) {
  std::string Filename;
  is >> nrec >> Filename >> MP_id;
  nstep = nrec;

  filename = box->result_folder + fileTool::separator() + Filename;
  std::cout << "DPCPlotSingle: filename is " << filename << std::endl;
  std::cout << "DPCPlotSingle: Tracked MP initial coordinates : " << box->MP[MP_id].pos.x << ", " << box->MP[MP_id].pos.y << std::endl;
  file.open(filename.c_str());
  
}

void DPCPlotSingle::exec() {
  MPStress.reset();
  MPStrain.reset();
  size_t nbMP = box->MP.size();
  if (0 == nbMP) {return;}
 
  MPStress = box->MP[MP_id].stress;
  MPStrain = box->MP[MP_id].strain;

  double diff_xx_yy = MPStress.xx-MPStress.yy;
  P = -0.5*(MPStress.xx + MPStress.yy);
  Q = sqrt(3*(0.25*diff_xx_yy*diff_xx_yy + MPStress.xy*MPStress.xy));

}

void DPCPlotSingle::record() {
  file << std::scientific << std::setprecision(std::numeric_limits<double>::digits10 + 1);
  file << box->t << ' ' << P << ' ' << Q << std::endl;  
}

void DPCPlotSingle::end() { file.close(); }

