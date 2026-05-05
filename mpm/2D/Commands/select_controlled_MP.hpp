#pragma once

#include "Core/ControlMP.hpp"

#include "Command.hpp"

#include <string>
#include <vector>

#include "ElementSelector.hpp"

struct select_controlled_MP : public Command {
	select_controlled_MP();
  void read(std::istream& is);
  void exec();

 private:
  ControlMP Ctr;
	
	ElementSelector<MPMbox> MP_Selector;
};
