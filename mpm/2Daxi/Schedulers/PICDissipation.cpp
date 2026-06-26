#include "PICDissipation.hpp"

#include "Core/MPMbox.hpp"

void PICDissipation::read(std::istream& is) {
  is >> box->ratioFLIP >> endTime;
	box->activePIC = true;
}

void PICDissipation::write(std::ostream& os) {
  os << "PICDissipation " << box->ratioFLIP << ' ' << endTime << '\n';
}

/**
 * @brief Check if the PIC damping should be activated or not.
 *
 * If the PIC damping has been activated, it checks if the current time is
 * greater than the end time specified in the configuration file. If it is,
 * the PIC damping is deactivated and a message is printed to the log.
 */
void PICDissipation::check() {
  if (box->activePIC == true) {
    box->activePIC = endTime > box->t;
    if (box->activePIC == false) {
      Logger::info("End of PIC damping at time {}", box->t);
    }
  } 
}
