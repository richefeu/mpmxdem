#include "Obstacle.hpp"

bool Obstacle::MPisInside(MaterialPoint &) {return false;}
// To instanciate an obstacle:
// string token ...
// Obstacle * obs = Factory<Obstacle>::Instance()->Create(token);

//Constructor
Obstacle::Obstacle(): isFree(false), pos(), vel(), acc(), force() {}

//Destructor
Obstacle::~Obstacle() {}

void Obstacle::deleteIfInside(MPMbox & /*MPM*/) {}
