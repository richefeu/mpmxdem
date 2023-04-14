#ifndef SPY_HPP
#define SPY_HPP

class MPMbox;
#include <fstream>
//#include <string>

struct Spy {
  MPMbox* box;

  int nstep;  // Period for exec
  int nrec;   // Period for record

  virtual void plug(MPMbox* Box);

  virtual void read(std::istream& is) = 0;

  virtual void exec() = 0;
  virtual void record() = 0;
  virtual void end() = 0;  // Called after all steps have been done

  virtual ~Spy();  // Dtor
};

#endif /* end of include guard: SPY_HPP */
