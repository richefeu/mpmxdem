#include <string>

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"
#include "select_controlled_MP.hpp"

#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

void select_controlled_MP::read(std::istream& is) {

  is >> Ctr.xcontrol >> Ctr.xvalue;
  is >> Ctr.ycontrol >> Ctr.yvalue;

  is >> inputOption;
  if (inputOption == "ALL") {

    box->console->info("@select_controlled_MP::read(), inputOption: '{}'", inputOption);

  } else if (inputOption == "ID") {

    box->console->info("@select_controlled_MP::read(), inputOption: '{}'", inputOption);
    is >> MP_ID;

  } else if (inputOption == "LIST") {

    box->console->info("@select_controlled_MP::read(), inputOption: '{}'", inputOption);
    size_t nb = 0;
    is >> nb;
    MP_ID_LIST.clear();
    for (size_t i = 0; i < nb; i++) {
      size_t ID;
      is >> ID;
      MP_ID_LIST.push_back(ID);
    }

  } else if (inputOption == "GRID") {

    box->console->info("@select_tracked_MP::read(), inputOption: '{}'", inputOption);
    is >> GRID_X0 >> GRID_Y0 >> GRID_LX >> GRID_LY;

  } else if (inputOption == "BOX") {

    box->console->info("@select_tracked_MP::read(), inputOption: '{}'", inputOption);
    is >> BOX_X0 >> BOX_Y0 >> BOX_X1 >> BOX_Y1;

  } else {

    box->console->error("@select_controlled_MP::read(), inputOption: '{}' is not known", inputOption);
  }
}

void select_controlled_MP::exec() {
  if (box->MP.empty()) {
    box->console->info("@select_controlled_MP::exec(), No MP has been added!");
  }

  if (inputOption == "ALL") {

    for (size_t p = 0; p < box->MP.size(); p++) {
      Ctr.PointNumber = p;
      box->controlledMP.push_back(Ctr);
    }

  } else if (inputOption == "ID") {

    if (MP_ID < box->MP.size()) {
      Ctr.PointNumber = MP_ID;
      box->controlledMP.push_back(Ctr);
    }

  } else if (inputOption == "LIST") {

    for (size_t i = 0; i < MP_ID_LIST.size(); i++) {
      if (MP_ID_LIST[i] < box->MP.size()) {
        Ctr.PointNumber = MP_ID_LIST[i];
        box->controlledMP.push_back(Ctr);
      }
    }

  } else if (inputOption == "GRID") {

    for (size_t p = 0; p < box->MP.size(); p++) {
      double xMP = box->MP[p].pos.x;
      double yMP = box->MP[p].pos.y;
      BOX_X0 = round((xMP - GRID_X0) / GRID_LX) - GRID_TOL;
      BOX_Y0 = round((yMP - GRID_Y0) / GRID_LY) - GRID_TOL;
      BOX_X1 = BOX_X0 + 2.0 * GRID_TOL;
      BOX_Y1 = BOX_Y0 + 2.0 * GRID_TOL;
      if (xMP >= BOX_X0 && xMP <= BOX_X1 && yMP >= BOX_Y0 && yMP <= BOX_Y1) {
        Ctr.PointNumber = p;
        box->controlledMP.push_back(Ctr);
      }
    }

  } else if (inputOption == "BOX") {

    for (size_t p = 0; p < box->MP.size(); p++) {
      double xMP = box->MP[p].pos.x;
      double yMP = box->MP[p].pos.y;
      if (xMP >= BOX_X0 && xMP <= BOX_X1 && yMP >= BOX_Y0 && yMP <= BOX_Y1) {
        Ctr.PointNumber = p;
        box->controlledMP.push_back(Ctr);
      }
    }
  }
}
