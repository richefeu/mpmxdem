#ifndef VTKOUTPUT_HPP
#define VTKOUTPUT_HPP

class MPMbox;
#include <fstream>

struct VtkOutput {
  MPMbox* box;

  virtual void plug(MPMbox* Box);
  virtual void read(std::istream& is);

  virtual void save(std::ostream& os) = 0;

  virtual ~VtkOutput();  // Dtor
};

#endif /* end of include guard: VTKOUTPUT_HPP */
