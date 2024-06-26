#ifndef MP_TRACKING_HPP
#define MP_TRACKING_HPP

#include "Spy.hpp"

#include <string>

#include "ElementSelector.hpp"
#include "mat4.hpp"

struct MPTracking : public Spy {
  MPTracking();
  void read(std::istream& is);
  void exec();
  void record();
  void end();

 private:
  ElementSelector<MPMbox> MP_Selector;
  std::vector<size_t> MP_ids;
  std::string filename;
  std::ofstream file;
  mat4r meanStress;
  mat4r meanStrain;
};

#endif /* end of include guard: MP_TRACKING_HPP */
