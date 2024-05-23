#include "PICDissipation.hpp"

#include "Core/MPMbox.hpp"

//#include "spdlog/sinks/stdout_color_sinks.h"
//#include "spdlog/spdlog.h"

void PICDissipation::read(std::istream& is) {
  is >> box->ratioFLIP >> endTime;
	box->activePIC = true;
}

void PICDissipation::write(std::ostream& os) {
  os << "PICDissipation " << box->ratioFLIP << ' ' << endTime << '\n';
}

void PICDissipation::check() {
  if (box->activePIC == true) {
    box->activePIC = endTime > box->t;
    if (box->activePIC == false) {
      Logger::info("End of PIC damping at time {}", box->t);
    }
  } 
}
