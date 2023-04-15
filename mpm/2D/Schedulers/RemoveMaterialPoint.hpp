#ifndef REMOVE_MP_HPP
#define REMOVE_MP_HPP

#include "Scheduler.hpp"

#include <string>

#include "vec2.hpp"

struct RemoveMaterialPoint : public Scheduler {

  void read(std::istream& is);
  void write(std::ostream& os);
  void check();

 private:
  std::string CMkey;  // this is the key-name given to a ConstitutiveModel
  double removeTime;  // time of removal
};

#endif /* end of include guard: REMOVE_MP_HPP */
