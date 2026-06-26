#pragma once

class MPMbox;
#include <fstream>

struct VtkOutput {
  MPMbox* box;

  virtual void plug(MPMbox* Box);
  virtual void read(std::istream& is);

  virtual void save(std::ostream& os) = 0;

  virtual ~VtkOutput();  // Dtor
};
