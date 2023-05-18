#ifndef MP_TRACKING_HPP
#define MP_TRACKING_HPP

#include "Spy.hpp"

#include <string>

#include "mat4.hpp"
#include "ElementSelector.hpp"

struct MPTracking : public Spy {
  void read(std::istream& is);

  void exec();
  void record();
  void end();

 private:
	//ElementSelector<MPMbox> selector;
	 std::vector<size_t> MP_ids;
  std::string filename;
  std::ofstream file;
	mat4r meanStress;
	mat4r meanStrain;
};

#endif /* end of include guard: MP_TRACKING_HPP */
