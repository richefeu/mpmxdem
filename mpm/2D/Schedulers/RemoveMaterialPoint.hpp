#pragma once

#include "Scheduler.hpp"

#include <string>

#include "vec2.hpp"

struct RemoveMaterialPoint : public Scheduler {

  void read(std::istream& is);
  void write(std::ostream& os);
  void check();

 private:
  std::string CMkey;  // this is the key-name given to a ConstitutiveModel
  double removeTime{0.0};  // time of removal
};
