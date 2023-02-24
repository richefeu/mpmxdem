#include <string>

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"
#include "select_tracked_MP.hpp"

#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

void select_tracked_MP::read(std::istream& is) {

  is >> inputOption;
  if (inputOption == "NONE") {
    box->console->info("@select_tracked_MP::read(), inputOption: '{}'", inputOption);
  } else if (inputOption == "ALL") {
    box->console->info("@select_tracked_MP::read(), inputOption: '{}'", inputOption);
  } else if (inputOption == "ID") {
    box->console->info("@select_tracked_MP::read(), inputOption: '{}'", inputOption);
    is >> MP_ID;
  } else if (inputOption == "LIST") {
    box->console->info("@select_tracked_MP::read(), inputOption: '{}'", inputOption);
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
    // TODO
  } else if (inputOption == "BOX") {
    box->console->info("@select_tracked_MP::read(), inputOption: '{}'", inputOption);
    // TODO
  } else {
    box->console->error("@select_tracked_MP::read(), inputOption: '{}' is not known", inputOption);
  }
}

void select_tracked_MP::exec() {
  if (box->MP.empty()) {
    box->console->info("@select_tracked_MP::exec(), No MP has been added!");
    // exit(0);
  }

  if (inputOption == "NONE") {
    for (size_t p = 0; p < box->MP.size(); p++) {
      box->MP[p].isTracked = false;
    }
  } else if (inputOption == "ALL") {
    for (size_t p = 0; p < box->MP.size(); p++) {
      box->MP[p].isTracked = true;
    }
  } else if (inputOption == "ID") {
    if (MP_ID < box->MP.size()) box->MP[MP_ID].isTracked = true;
  } else if (inputOption == "LIST") {
    for (size_t i = 0; i < MP_ID_LIST.size(); i++) {
      if (MP_ID_LIST[i] < box->MP.size()) box->MP[MP_ID_LIST[i]].isTracked = true;
    }
  } else if (inputOption == "GRID") {
    // TODO
  } else if (inputOption == "BOX") {
    // TODO
  }
}
