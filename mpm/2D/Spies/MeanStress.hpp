#ifndef MEAN_STRESS_HPP
#define MEAN_STRESS_HPP

#include "Spy.hpp"

#include <string>

#include "mat4.hpp"

struct MeanStress : public Spy {
  void read(std::istream& is);

  void exec();
  void record();
  void end();

 private:
  std::string filename;
  std::ofstream file;
	mat4r meanStress;
};

#endif /* end of include guard: MEAN_STRESS_HPP */
