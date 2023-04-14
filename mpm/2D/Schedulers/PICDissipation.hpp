#ifndef PIC_DISSIPATION_HPP
#define PIC_DISSIPATION_HPP

#include "Scheduler.hpp"

struct PICDissipation : public Scheduler {
	
  void read(std::istream& is);
	void write(std::ostream& os);
  void check();
  
 private:
  double endTime;
};


#endif /* end of include guard: PIC_DISSIPATION_HPP */
