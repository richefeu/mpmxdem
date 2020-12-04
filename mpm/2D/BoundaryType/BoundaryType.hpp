#ifndef BOUNDARYTYPE_HPP_DEEAEE92
#define BOUNDARYTYPE_HPP_DEEAEE92

#include <Core/Neighbor.hpp>
#include <DataTable.hpp>
#include <Obstacles/Obstacle.hpp>
#include <iostream>
#include <vec2.hpp>
#include <vector>

struct MPMbox;
struct MaterialPoint;
struct Obstacle;
struct BoundaryType {
  virtual void calculateContactForces(std::vector<MaterialPoint>& MP, DataTable dataTable, Obstacle* currentObstacle,
                                      size_t id_kn, size_t id_kt, size_t id_en2, size_t id_mu, size_t viscosity,
                                      double dt) = 0;
  BoundaryType();
  virtual ~BoundaryType();
};

#endif /* end of include guard: BOUNDARYTYPE_HPP_DEEAEE92 */
