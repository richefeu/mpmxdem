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
  mat9r f;     ///< Imposed strain

  char StoredCommand[512];

  // This function will be set to a lambda (c++11)
  // It can be used for 'smart' driving conditions like high dependent velocity or complexe loading
  // such as cyclic loading or loading for building a yield surface.
  std::function<void(PBC3Dbox &box)> ServoFunction=nullptr; ///< This lamda function is used for 'smart' driving conditions

  Loading();

  void TriaxialCompressionY(double pressure, double velocity);
  void TriaxialCompressionZ(double pressure, double velocity);
  void BiaxialCompressionYPlaneStrainZ(double pressure, double velocity);
  void BiaxialCompressionZPlaneStrainX(double pressure, double velocity);
  void BiaxialCompressionXPlaneStrainY(double pressure, double velocity);
  void BiaxialCompressionY(double px, double py);
  void IsostaticCompression(double pressure);
  void SimpleShearXY(double pressure, double gammaDot);
  void VelocityControl(mat9r &V);
  void VelocityControlPlaneStress(mat9r &V,double &pressure);
  void RigidRotationZ(double omega);
  void StrainControl(mat9r &F);
  void TransformationGradient(mat9r &h, mat9r &F, double duration);
  void AxisRotationZ(double E0, double omega, double Lx, double Ly, double iniTime);
  void LodeAnglePath(double pressure, double velocity, double LodeAngle);
  void LodeAnglePathMix(double pressure, double velocity, double LodeAngle);
  void Fixe();
};

#endif /* end of include guard: LOADING_HPP */
