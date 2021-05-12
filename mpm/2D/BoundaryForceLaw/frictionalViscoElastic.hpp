#ifndef KELVINVOIGT_HPP_C1EB5D8D
#define KELVINVOIGT_HPP_C1EB5D8D

#include "BoundaryType.hpp"

struct KelvinVoigt : public BoundaryType {
  virtual void calculateContactForces(std::vector<MaterialPoint>& MP, DataTable dataTable, Obstacle* currentObstacle,
                                      size_t id_kn, size_t id_kt, size_t id_en2, size_t id_mu, size_t viscosity,
                                      double dt);
};

#endif /* end of include guard: KELVINVOIGT_HPP_C1EB5D8D */
