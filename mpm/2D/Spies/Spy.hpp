#pragma once

class MPMbox;
#include <fstream>

struct Spy {
  MPMbox* box{nullptr};

  // Both are used as a modulo in MPMbox::run(), so they must never be zero.
  // MPMbox::checkSettings() refuses to start if a Spy leaves them at 0 or below.
  int nstep{1};  // Period for exec
  int nrec{1};   // Period for record

  virtual void plug(MPMbox* Box);

  virtual void read(std::istream& is) = 0;

  virtual void exec() = 0;
  virtual void record() = 0;
  virtual void end() = 0;  // Called after all steps have been done

  virtual ~Spy();  // Dtor
};
