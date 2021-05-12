#ifndef ONESTEP_HPP
#define ONESTEP_HPP

#include <fstream>

#include <Core/MaterialPoint.hpp>
#include <DataTable.hpp>
#include <Obstacles/Obstacle.hpp>

struct MPMbox;

struct OneStep {
  MPMbox* box;

  virtual void plug(MPMbox* Box);

  virtual int advanceOneStep(MPMbox& MPM) = 0;
  void resetDEM(Obstacle* obst, vec2r gravity);
  void moveDEM1(Obstacle* obst, double dt, bool activeNumericalDissipation);
  void moveDEM2(Obstacle* obst, double dt);
  vec2r numericalDissipation(vec2r velMP, vec2r forceMP);
  // void boundaryConditions(std::vector<MaterialPoint> &MP, std::vector<Obstacle*> &Obstacles,
  // DataTable dataTable, size_t id_kn, size_t id_kt, size_t id_en2, size_t id_mu, double dt);

  virtual ~OneStep();  // Dtor
};

#endif /* end of include guard: ONESTEP_HPP */
