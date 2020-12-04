#include "blender_Obstacle.hpp"

bool blender_Obstacle::MPisInside(MaterialPoint &) {return false;}
// To instanciate an obstacle:
// string token ...
// Obstacle * obs = Factory<Obstacle>::Instance()->Create(token);

//Constructor
blender_Obstacle::blender_Obstacle(): isFree(false), pos(), vel(), acc(), force() {}

//Destructor
blender_Obstacle::~blender_Obstacle() {}
