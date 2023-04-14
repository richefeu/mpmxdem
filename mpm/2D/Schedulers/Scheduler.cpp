#include "Scheduler.hpp"

Scheduler::~Scheduler() {}

void Scheduler::plug(MPMbox* Box) { box = Box; }
