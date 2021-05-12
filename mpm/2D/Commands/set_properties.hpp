#ifndef SET_PROPERTIES_HPP
#define SET_PROPERTIES_HPP

#include "Command.hpp"

struct set_properties : public Command {
  void read(std::istream& is);
  void exec();

 private:
  double kn, kt;
  double en2, mu;
};

#endif /* end of include guard: SET_PROPERTIES_HPP */
