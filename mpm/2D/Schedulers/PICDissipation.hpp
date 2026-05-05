#pragma once

#include "Scheduler.hpp"

struct PICDissipation : public Scheduler {
	
  void read(std::istream& is);
	void write(std::ostream& os);
  void check();
  
 private:
  double endTime;
};
