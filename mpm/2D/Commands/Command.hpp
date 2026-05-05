#pragma once

#include <fstream>
class MPMbox;

struct Command {
  MPMbox* box;

  virtual void plug(MPMbox* Box);

  virtual void read(std::istream& is) = 0;
  virtual void exec() = 0;

  virtual ~Command();  // Dtor
};

