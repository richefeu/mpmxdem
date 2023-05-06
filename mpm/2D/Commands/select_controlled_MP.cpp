#include <string>

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"
#include "select_controlled_MP.hpp"

#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

select_controlled_MP::select_controlled_MP() {
  MP_Selector.actionForNone = [this](MPMbox* B) { B->controlledMP.clear(); };

  MP_Selector.actionForId = [this](MPMbox* B, size_t p) {
    this->Ctr.PointNumber = p;
    B->controlledMP.push_back(Ctr);
  };

  MP_Selector.ContainerSize = [](MPMbox* B) -> size_t { return B->MP.size(); };

  MP_Selector.getXY = [](MPMbox* B, size_t p, double& x, double& y) {
    x = B->MP[p].pos.x;
    y = B->MP[p].pos.y;
  };
}

void select_controlled_MP::read(std::istream& is) {

  is >> Ctr.xcontrol >> Ctr.xvalue;
  is >> Ctr.ycontrol >> Ctr.yvalue;

  is >> MP_Selector;
}

void select_controlled_MP::exec() {
  if (box->MP.empty()) {
    box->console->info("@select_controlled_MP::exec(), No MP has been added!");
  }

  MP_Selector.execute(box);
}
