#ifndef PIC_DISSIPATION_BY_PIC_HPP
#define PIC_DISSIPATION_BY_PIC_HPP

#include "Scheduler.hpp"

#include <vector>

struct PICDissipationByPIC : public Scheduler {
  void read(std::istream &is) override;
  void write(std::ostream &os) override;
  void check() override;

private:
  // Piecewise-constant PIC ratio defined by a list of time stamps.
  // For t < times[0]: do nothing. For t in [times[i], times[i+1]): impose ratioPIC[i].
  // For t >= times.back(): impose ratioPIC.back().
  std::vector<double> times;
  std::vector<double> ratioPIC;
};

#endif /* end of include guard: PIC_DISSIPATION_BY_PIC_HPP */
