#include "Obstacle.hpp"
#include "factory.hpp"

bool Obstacle::inside(vec2r&) { return false; }

//
// Carry the contact history over a rebuild of the neighbour list.
//
// Every proxPeriod steps, checkProximity throws the list away and builds a new
// one. The tangential force ft, the overlap dn and the normal stress sigma_n
// are however accumulated from one step to the next: losing them means the
// contact starts sliding again from scratch, which shows up as a block slowly
// creeping down a slope it should be holding on to.
//
// Both lists are sorted by increasing PointNumber -- they are built by scanning
// the Material Points in order -- so a single merge pass is enough. The whole
// Neighbor is copied, and not only fn and ft: dn drives the loading/unloading
// branch of frictionalNormalRestitution, sigma_n the threshold of
// frictionalViscoElastofragile, and dn is what the Work and EnergyBalance spies
// use to tell an active contact from an inactive one.
//
void Obstacle::restoreNeighborHistory(const std::vector<Neighbor>& previous) {
  size_t istore = 0;
  for (size_t inew = 0; inew < Neighbors.size(); inew++) {

    // Advance in the old list as long as it lags behind the new one. Comparing
    // the other way round -- which is what the three obstacles used to do --
    // works only as long as the two lists are identical: as soon as a point
    // leaves the list, the two cursors never meet again and the history of
    // every following contact is silently dropped.
    while (istore < previous.size() && previous[istore].PointNumber < Neighbors[inew].PointNumber) { ++istore; }

    if (istore == previous.size()) { break; }

    if (previous[istore].PointNumber == Neighbors[inew].PointNumber) {
      Neighbors[inew] = previous[istore];
      ++istore;
    }
  }
}

// Ctor
Obstacle::Obstacle() : isFree(false), pos(), vel(), acc(), force() {
  std::string defaultBoundary = "frictionalNormalRestitution";
  boundaryForceLaw = Factory<BoundaryForceLaw>::Instance()->Create(defaultBoundary);
}

// Dtor
Obstacle::~Obstacle() {}
