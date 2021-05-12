#ifndef INCREMENTALLINEAR_HPP_C1EB5D8D
#define INCREMENTALLINEAR_HPP_C1EB5D8D

#include "BoundaryType.hpp"

struct IncrementalLinear : public BoundaryType {
  virtual void calculateContactForces(std::vector<MaterialPoint>& MP, DataTable dataTable, Obstacle* currentObstacle,
                                      size_t id_kn, size_t id_kt, size_t id_en2, size_t id_mu, size_t viscosity,
                                      double dt);
};

#endif /* end of include guard: INCREMENTALLINEAR_HPP_C1EB5D8D */
