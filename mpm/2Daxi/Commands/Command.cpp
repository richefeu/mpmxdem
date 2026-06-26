#include "Command.hpp"

Command::~Command() {}

void Command::plug(MPMbox* Box) { box = Box; }
