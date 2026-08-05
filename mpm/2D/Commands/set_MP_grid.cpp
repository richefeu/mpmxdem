#include "set_MP_grid.hpp"
#include "ConstitutiveModels/ConstitutiveModel.hpp"
#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

//#include "spdlog/sinks/stdout_color_sinks.h"
//#include "spdlog/spdlog.h"

void set_MP_grid::read(std::istream& is) { is >> groupNb >> modelName >> rho >> x0 >> y0 >> x1 >> y1 >> size; }

void set_MP_grid::exec() {
  if (size <= 0.0) {
    Logger::critical("@set_MP_grid::exec, the MP size has to be positive (given: {})", size);
    exit(EXIT_FAILURE);
  }
  if (box->Grid.lx / size < 2.0 || box->Grid.ly / size < 2.0) {
    Logger::critical("@set_MP_grid::exec, the MP size ({}) is too large for the grid cells ({} x {}): "
                     "there has to be at least 2 points per cell in each direction",
                     size, box->Grid.lx, box->Grid.ly);
    exit(EXIT_FAILURE);
  }

  auto itCM = box->models.find(modelName);
  if (itCM == box->models.end()) {
    Logger::critical("@set_MP_grid::exec, the model '{}' is not defined", modelName);
    Logger::critical("  A 'model' line has to declare it before this command");
    exit(EXIT_FAILURE);
  }
  ConstitutiveModel* CM = itCM->second;

  double halfSizeMP = 0.5 * size;

  int counter = 0;

  // new loop 15/05/2018 (bug should still exist but its working better now)
  double nbMPX = (x1 - x0) / size;
  double nbMPY = (y1 - y0) / size;

  // https://stackoverflow.com/questions/9695329/c-how-to-round-a-double-to-an-int
  nbMPX += 0.5;
  nbMPY += 0.5;
  nbMPX = (int)nbMPX;
  nbMPY = (int)nbMPY;
  
  Logger::info("@set_MP_grid::exec, nbMPX = {}, nbMPY = {}", nbMPX, nbMPY);
  for (int i = 0; i < nbMPY; i++) {
    for (int j = 0; j < nbMPX; j++) {
      MaterialPoint P(groupNb, size, rho, CM);
      CM->init(P);
      if (P.isDoubleScale == true) {
        double Vcell = fabs(P.PBC->Cell.h.det());
        //P.density = P.PBC->Cell.mass / Vcell; 
				P.density = P.PBC->density * P.PBC->Vsolid / Vcell;        
      }
      P.pos.set(x0 + halfSizeMP + size * j, y0 + halfSizeMP + size * i);
      P.nb = counter;
      counter++;
      box->MP.push_back(P);
    }
  }

  for (size_t p = 0; p < box->MP.size(); p++) {
    box->MP[p].updateCornersFromF();
  }

  for (size_t p = 0; p < box->MP.size(); p++) {
    if (box->MP[p].pos.x > (double)box->Grid.Nx * box->Grid.lx || box->MP[p].pos.x < 0.0 ||
        box->MP[p].pos.y > (double)box->Grid.Ny * box->Grid.ly || box->MP[p].pos.y < 0.0) {
      Logger::critical("@set_MP_grid::exec, the Material Point {} at ({}, {}) is outside the grid [0, {}] x [0, {}]", p,
                       box->MP[p].pos.x, box->MP[p].pos.y, (double)box->Grid.Nx * box->Grid.lx,
                       (double)box->Grid.Ny * box->Grid.ly);
      exit(EXIT_FAILURE);
    }
  }
}
