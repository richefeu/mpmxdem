#include "VtkOutput.hpp"

VtkOutput::~VtkOutput() {}

void VtkOutput::plug(MPMbox* Box) { box = Box; }

void VtkOutput::read(std::istream&) {}
