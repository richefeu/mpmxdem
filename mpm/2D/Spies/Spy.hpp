#pragma once

class MPMbox;
#include <fstream>

struct Spy {
  MPMbox* box{nullptr};

  int nstep{0};  // Period for exec
  int nrec{0};   // Period for record

  virtual void plug(MPMbox* Box);

  virtual void read(std::istream& is) = 0;

  virtual void exec() = 0;
  virtual void record() = 0;
  virtual void end() = 0;  // Called after all steps have been done

  virtual ~Spy();  // Dtor
};
