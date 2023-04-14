#ifndef SCHEDULER_HPP
#define SCHEDULER_HPP

class MPMbox;
#include <fstream>

struct Scheduler {
	MPMbox* box;
	
	virtual void plug(MPMbox* Box);
	
	virtual void read(std::istream& is) = 0;
	virtual void write(std::ostream& os) = 0;
	virtual void check() = 0;
	
	virtual ~Scheduler ();
};



#endif /* end of include guard: SCHEDULER_HPP */
