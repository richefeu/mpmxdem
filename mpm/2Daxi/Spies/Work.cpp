#include "Work.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"
#include "Obstacles/Obstacle.hpp"

#include "fileTool.hpp"

void Work::read(std::istream &is) {
  start = true;

  std::string Filename;
  is >> plotType >> nrec >> Filename;
  nstep = 1; // exec is called at each time step
  is >> Xmin >> Xmax >> nbSlices;
  if (plotType == "hist") {
  filenameSlices = box->result_folder + fileTool::separator() + "hist" + fileTool::GetFileName(Filename) + std::to_string(nbSlices) + "Slices." +
                    fileTool::GetFileExt(Filename);
  }
  else if (plotType == "line") {
  filenameSlices = box->result_folder + fileTool::separator() + "line" + fileTool::GetFileName(Filename) + std::to_string(nbSlices) + "Slices." +
                    fileTool::GetFileExt(Filename);
  }
  filename       = box->result_folder + fileTool::separator()+ Filename;
  std::cout << "WorkSlice: filename is " << filenameSlices << std::endl;
  std::cout << "Work: filename is " << filename << std::endl;
  
  // Only open file in computation mode to prevent overwriting during visualization
  if (box->computationMode) { fileSlices.open(filenameSlices.c_str()); }
  // Only open file in computation mode to prevent overwriting during visualization
  if (box->computationMode) { file.open(filename.c_str()); }

  
  Range.set(Xmin, Xmax, nbSlices);
  Wn.resize(nbSlices);
  Wt.resize(nbSlices);
  Wint.resize(nbSlices);
  }

void Work::exec() {
  double MP_Wn;
  double MP_Wt;
  double MP_Wint;
  double MP_Wp;
  double MP_KEr;
  int islice;  
  for (size_t p = 0; p < box->MP.size(); p++) {
    islice = Range.getID(box->MP[p].pos.x);
    if (islice < 0) continue; // It means 'out-of-slice'
    // Internal work
    MP_Wint =
        box->MP[p].vol * (box->MP[p].stress.xx * box->MP[p].deltaStrain.xx + box->MP[p].stress.yy * box->MP[p].deltaStrain.yy +
                          2.0 * box->MP[p].stress.xy * box->MP[p].deltaStrain.xy);
    Wint[islice] += MP_Wint;
    Wint_tot += abs(MP_Wint);

    // Weight
    MP_Wp = box->MP[p].mass * box->gravity.y * (box->MP[p].pos - box->MP[p].prev_pos) * vec2r::unit_y();
    // if (p==box->MP.size()-1) {
    //   std::cout<<"Work : "<<box->MP[p].prev_pos<<" "<<box->MP[p].pos<<std::endl;
    //   }
    Wp_tot += abs(MP_Wp);

    // Kinetic Energy rate
    
    MP_KEr = 0.5 * box -> MP[p].mass * ( (box->MP[p].vel*box->MP[p].vel) - (box->MP[p].prev_vel*box->MP[p].prev_vel));
    KEr_tot += abs(MP_KEr);
  }
  
  // MP Works due to forces with obstacles); ++o) {
  
  for (size_t o = 0; o < box->Obstacles.size(); ++o){
    for (size_t nn = 0; nn < box->Obstacles[o]->Neighbors.size(); ++nn) {
      if (box->Obstacles[o]->Neighbors[nn].dn >= 0.0) continue;
      size_t pn = box->Obstacles[o]->Neighbors[nn].PointNumber;

      islice = Range.getID(box->MP[pn].pos.x);
      if (islice < 0) continue; // It means 'out-of-slice'

      vec2r disp = box->MP[pn].pos - box->MP[pn].prev_pos; // here the obstacle is not supposed to move
      vec2r N, T;
      box->Obstacles[o]->getContactFrame(box->MP[pn], N, T);
      double delta_dn = disp * N;
      double delta_dt = disp * T;

      MP_Wn = delta_dn * box->Obstacles[o]->Neighbors[nn].fn;
      Wn[islice] += MP_Wn;
      Wn_tot += abs(MP_Wn);

      MP_Wt = delta_dt * box->Obstacles[o]->Neighbors[nn].ft;
      Wt[islice] += MP_Wt;
      Wt_tot += abs(MP_Wt);
    
    }
  }
}

void Work::record() {
  // Only record if file is open (i.e., if we're in computation mode)
  if (!file.is_open()) return;
  if (start) {
    file << "#temps  Wn_tot  -Wt_tot  Wp_tot  Wint_tot  KEr_tot  " << std::endl;
    start = false;
  }
  file << box->t << " " << Wn_tot << " " << Wt_tot << " "  << Wp_tot << " "  << Wint_tot << " " << KEr_tot << std::endl;
}

void Work::end() {
  double bin  = Range.getStep();
  double vmin = Range.getLeftValue();
  double vmax = Range.getRightValue();
  if (plotType == "hist") {
    fileSlices << vmin << " " << 0 << " " << 0 << " " << 0 << std::endl;
    for (int i = 0; i < Range.getNumberOfSlices(); i++) {
    fileSlices << vmin + i * bin << " " << 0 << " " << 0 << " " << 0 << std::endl;
    fileSlices << vmin + i * bin << " " << -Wn[i]/bin << " " << -Wt[i]/bin << " "  << Wint[i]/bin << std::endl;
      if (i<Range.getNumberOfSlices()-1) {
      fileSlices << vmin + (i + 1) * bin << " " << -Wn[i]/bin << " " << -Wt[i]/bin << " " << Wint[i]/bin << std::endl;
      }
    }
    fileSlices << vmax << " " << 0 << " " << 0 << " " << 0 << std::endl;
  }
    else if (plotType == "line") {
      for (int i = 0; i < Range.getNumberOfSlices(); i++) {
    fileSlices << vmin + (i + 1) * bin << " " << -Wn[i]/bin << " " << -Wt[i]/bin << " " << Wint[i]/bin << std::endl;
    }
    }
  fileSlices.close();
  file.close();
}