#pragma once

#include "Scheduler.hpp"

#include "vec2.hpp"

struct RemoveObstacle : public Scheduler {
	
  void read(std::istream& is);
	void write(std::ostream& os);
  void check();
  
 private:
   int groupNumber{0};   // osbsacle group-number to be suppressed
   double removeTime{0.0}; // time to remove the obstacle(s)
};
