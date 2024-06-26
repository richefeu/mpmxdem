#ifndef COMMAND_HPP_64744727
#define COMMAND_HPP_64744727

#include <fstream>
#include "vec2.hpp"
#include "vec3.hpp"
#include <vector>
struct MPMbox;

struct Command
{
	MPMbox *box;

	virtual void plug(MPMbox *Box);

	virtual void read(std::istream & is) = 0;
	virtual void exec() = 0;

	virtual ~Command(); // Dtor
};

#endif /* end of include guard: COMMAND_HPP_64744727 */
