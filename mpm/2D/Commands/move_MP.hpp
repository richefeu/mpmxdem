#ifndef MOVE_MP_HPP_2C140B65
#define MOVE_MP_HPP_2C140B65

#include "Command.hpp"

struct move_MP : public Command {
  void read(std::istream& is);
  void exec();

 private:
  int groupNb;
  double x0;
  double y0;
  double dx;
  double dy;
  double thetaDeg;
};

#endif /* end of include guard: MOVE_MP_HPP_2C140B65 */
