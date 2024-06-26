#ifndef LOADING_HPP
#define LOADING_HPP

#include <functional>

#include "mat9.hpp"

class PBC3Dbox;

const bool ForceDriven = true;
const bool VelocityDriven = false;

/// Loading applied to collective degrees of freedom (DoF)
struct Loading {
  mat9b Drive; ///< Driving mode. Can be ForceDriven or VelocityDriven
  mat9r Sig;   ///< Imposed external stress
  mat9r v;     ///< Imposed velocities

  char StoredCommand[256];

  // This function will be set to a lambda (c++11)
  // It can be used for 'smart' driving conditions like high dependent velocity or complexe loading
  // such as cyclic loading or loading for building a yield surface.
  std::function<void(PBC3Dbox &box)> ServoFunction; ///< This lamda function is used for 'smart' driving conditions

  Loading();

  void TriaxialCompressionY(double pressure, double velocity);
  void TriaxialCompressionZ(double pressure, double velocity);
  void BiaxialCompressionYPlaneStrainZ(double pressure, double velocity);
  void BiaxialCompressionZPlaneStrainX(double pressure, double velocity);
  void IsostaticCompression(double pressure);
  void SimpleShearXY(double pressure, double gammaDot);
  void VelocityControl(mat9r &V);
  void TransformationGradient(mat9r &h, mat9r &F, double duration);
  void LodeAnglePath(double pressure, double velocity, double LodeAngle);
  void LodeAnglePathMix(double pressure, double velocity, double LodeAngle);
  void Fixe();
};

#endif /* end of include guard: LOADING_HPP */
