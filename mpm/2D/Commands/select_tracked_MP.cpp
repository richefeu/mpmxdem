#include <string>

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"
#include "select_tracked_MP.hpp"

#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

select_tracked_MP::select_tracked_MP() {
  MP_Selector.actionForNone = [this](MPMbox* B) {
    for (size_t p = 0; p < B->MP.size(); p++) {
      B->MP[p].isTracked = false;
    }
  };

  MP_Selector.actionForId = [](MPMbox* B, size_t p) { B->MP[p].isTracked = true; };

  MP_Selector.ContainerSize = [](MPMbox* B) -> size_t { return B->MP.size(); };

  MP_Selector.getXY = [](MPMbox* B, size_t p, double& x, double& y) {
    x = B->MP[p].pos.x;
    y = B->MP[p].pos.y;
  };
}

void select_tracked_MP::read(std::istream& is) { is >> MP_Selector; }

void select_tracked_MP::exec() {
  if (box->MP.empty()) {
    box->console->info("@select_tracked_MP::exec(), No MP has been added!");
  }

  MP_Selector.execute(box);
}
