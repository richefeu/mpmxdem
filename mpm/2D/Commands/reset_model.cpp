#include "reset_model.hpp"

#include <ConstitutiveModels/ConstitutiveModel.hpp>
#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>

#include <factory.hpp>
static Registrar<Command, reset_model> registrar("reset_model");

void reset_model::read(std::istream& is) { is >> groupNb >> modelName >> rho >> x0 >> y0 >> x1 >> y1; }

void reset_model::exec() {
  auto itCM = box->models.find(modelName);
  if (itCM == box->models.end()) {
    std::cerr << "@add_MP_ShallowPath::exec, model " << modelName << " not found" << std::endl;
  }
  ConstitutiveModel* CM = itCM->second;

  for (size_t p = 0; p < box->MP.size(); p++) {
    if (box->MP[p].pos.x > x0 && box->MP[p].pos.x < x1) {
      if (box->MP[p].pos.y > y0 && box->MP[p].pos.y < y1) {
        box->MP[p].density = rho;
        box->MP[p].mass = box->MP[p].vol * rho;
        box->MP[p].constitutiveModel = CM;
        box->MP[p].groupNb = groupNb;
      }
    }
  }
}
