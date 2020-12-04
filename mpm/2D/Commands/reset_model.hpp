#ifndef RESET_MODEL_HPP_8105857F
#define RESET_MODEL_HPP_8105857F

#include <string>

#include "Command.hpp"

struct reset_model : public Command {
  void read(std::istream& is);
  void exec();

 private:
  std::string modelName;
  int groupNb;
  double rho, x0, y0, x1, y1;
};

#endif /* end of include guard: RESET_MODEL_HPP_8105857F */
