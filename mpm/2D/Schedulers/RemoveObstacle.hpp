#pragma once

#include "Scheduler.hpp"

#include "vec2.hpp"

struct RemoveObstacle : public Scheduler {
	
  void read(std::istream& is);
	void write(std::ostream& os);
  void check();
  
 private:
   int groupNumber;   // osbsacle group-number to be suppressed
   double removeTime; // time to remove the obstacle(s)
};
