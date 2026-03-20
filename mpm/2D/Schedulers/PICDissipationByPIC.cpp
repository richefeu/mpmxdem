#include "PICDissipationByPIC.hpp"

#include "Core/MPMbox.hpp"

#include <sstream>

void PICDissipationByPIC::read(std::istream &is) {
  // Read the rest of the current line as a list of (time, ratioPIC) pairs.
  // Example:
  //   Scheduled PICDissipationByPIC 0 0.95 1.5 0.05
  std::string line;
  std::getline(is >> std::ws, line);

  std::istringstream ls(line);

  times.clear();
  ratioPIC.clear();

  double ti = 0.0;
  double pi = 0.0;
  while (ls >> ti >> pi) {
    times.push_back(ti);
    ratioPIC.push_back(pi);
  }

  if (!ls.eof()) {
    Logger::warn("PICDissipationByPIC: could not parse schedule line: '{}'", line);
  }

  if (times.empty()) {
    Logger::warn("PICDissipationByPIC: empty schedule (expected pairs: time ratioPIC)");
    return;
  }

  for (size_t i = 0; i < ratioPIC.size(); ++i) {
    if (ratioPIC[i] < 0.0 || ratioPIC[i] > 1.0) {
      Logger::warn("PICDissipationByPIC: PIC ratio {} should be in [0,1]", ratioPIC[i]);
    }
  }

  // Ensure PIC blending is evaluated; the actual on/off and ratio are imposed in check().
  box->activePIC = true;
}

void PICDissipationByPIC::write(std::ostream &os) {
  os << "PICDissipationByPIC";
  for (size_t i = 0; i < times.size(); ++i) {
    os << ' ' << times[i] << ' ' << ratioPIC[i];
  }
  os << '\n';
}

void PICDissipationByPIC::check() {
  const double t = box->t;

  if (times.empty()) { return; }

  // Before the first time mark: keep current settings.
  if (t < times.front()) { return; }

  // Find last i such that times[i] <= t.
  size_t idx = 0;
  for (size_t i = 1; i < times.size(); ++i) {
    if (t < times[i]) { break; }
    idx = i;
  }

  const double pic = ratioPIC[idx];
  box->ratioFLIP   = 1.0 - pic;
  box->activePIC   = (pic > 0.0);
}
