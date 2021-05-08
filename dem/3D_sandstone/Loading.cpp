#include "PBC3D.hpp"

Loading::Loading() {}

void Loading::TriaxialCompressionY(double pressure, double velocity) {
  sprintf(StoredCommand, "TriaxialCompressionY %g %g", pressure, velocity);

  Drive.xx = Drive.zz = ForceDriven;
  Drive.yy = VelocityDriven;
  Drive.xy = Drive.yx = VelocityDriven;
  Drive.xz = Drive.zx = VelocityDriven;
  Drive.yz = Drive.zy = VelocityDriven;

  Sig.xx = Sig.zz = pressure;
  Sig.yy = 0.0;
  Sig.xy = Sig.yx = 0.0;
  Sig.xz = Sig.yz = 0.0;
  Sig.yz = Sig.zy = 0.0;

  v.xx = v.zz = 0.0; // free in fact
  v.yy = -velocity;
  v.xy = v.yx = 0.0;
  v.xz = v.zx = 0.0;
  v.yz = v.zy = 0.0;

  ServoFunction = nullptr;
}

void Loading::TriaxialCompressionZ(double pressure, double velocity) {
  sprintf(StoredCommand, "TriaxialCompressionZ %g %g", pressure, velocity);

  Drive.xx = Drive.yy = ForceDriven;
  Drive.zz = VelocityDriven;
  Drive.xy = Drive.yx = VelocityDriven;
  Drive.xz = Drive.zx = VelocityDriven;
  Drive.yz = Drive.zy = VelocityDriven;

  Sig.xx = Sig.yy = pressure;
  Sig.zz = 0.0;
  Sig.xy = Sig.yx = 0.0;
  Sig.xz = Sig.yz = 0.0;
  Sig.yz = Sig.zy = 0.0;

  v.xx = v.yy = 0.0; // free in fact
  v.zz = -velocity;
  v.xy = v.yx = 0.0;
  v.xz = v.zx = 0.0;
  v.yz = v.zy = 0.0;

  ServoFunction = nullptr;
}

void Loading::BiaxialCompressionYPlaneStrainZ(double pressure, double velocity) {
  sprintf(StoredCommand, "BiaxialCompressionYPlaneStrainZ %g %g", pressure, velocity);

  Drive.xx = ForceDriven;    // Pressure applied along x
  Drive.yy = VelocityDriven; // Velocity imposed
  Drive.zz = VelocityDriven; // Plane Strain condition
  Drive.xy = Drive.yx = VelocityDriven;
  Drive.xz = Drive.zx = VelocityDriven;
  Drive.yz = Drive.zy = VelocityDriven;

  Sig.xx = pressure;
  Sig.yy = 0.0;
  Sig.zz = 0.0;
  Sig.xy = Sig.yx = 0.0;
  Sig.xz = Sig.yz = 0.0;
  Sig.yz = Sig.zy = 0.0;

  v.xx = 0.0; // free in fact
  v.yy = -velocity;
  v.zz = 0.0;
  v.xy = v.yx = 0.0;
  v.xz = v.zx = 0.0;
  v.yz = v.zy = 0.0;

  ServoFunction = nullptr;
}

void Loading::BiaxialCompressionZPlaneStrainX(double pressure, double velocity) {
  sprintf(StoredCommand, "BiaxialCompressionZPlaneStrainX %g %g", pressure, velocity);

  Drive.zz = VelocityDriven; // Plane Strain condition
  Drive.xx = ForceDriven;    // Pressure applied along y
  Drive.yy = VelocityDriven; // Velocity imposed

  Drive.xy = Drive.yx = VelocityDriven;
  Drive.xz = Drive.zx = VelocityDriven;
  Drive.yz = Drive.zy = VelocityDriven;

  Sig.zz = 0.0;
  Sig.xx = pressure;
  Sig.yy = 0.0;
  Sig.xy = Sig.yx = 0.0;
  Sig.xz = Sig.yz = 0.0;
  Sig.yz = Sig.zy = 0.0;

  v.zz = 0.0;
  v.xx = 0.0; // free in fact
  v.yy = -velocity;
  v.xy = v.yx = 0.0;
  v.xz = v.zx = 0.0;
  v.yz = v.zy = 0.0;

  ServoFunction = nullptr;
}

void Loading::IsostaticCompression(double pressure) {
  sprintf(StoredCommand, "IsostaticCompression %g", pressure);

  Drive.xx = Drive.yy = Drive.zz = ForceDriven;
  Drive.xy = Drive.yx = VelocityDriven;
  Drive.xz = Drive.zx = VelocityDriven;
  Drive.yz = Drive.zy = VelocityDriven;

  Sig.xx = Sig.yy = Sig.zz = pressure;
  Sig.xy = Sig.yx = 0.0;
  Sig.xz = Sig.yz = 0.0;
  Sig.yz = Sig.zy = 0.0;

  v.xx = v.yy = v.zz = 0.0; // free in fact
  v.xy = v.yx = 0.0;
  v.xz = v.zx = 0.0;
  v.yz = v.zy = 0.0;

  ServoFunction = nullptr;
}

// Shear in X direction while applying a pressure in Y direction
void Loading::SimpleShearXY(double pressure, double gammaDot) {
  sprintf(StoredCommand, "SimpleShearXY %g %g", pressure, gammaDot);

  Drive.yy = ForceDriven;
  Drive.xx = Drive.zz = VelocityDriven;
  Drive.xy = Drive.yx = VelocityDriven;
  Drive.xz = Drive.zx = VelocityDriven;
  Drive.yz = Drive.zy = VelocityDriven;

  Sig.yy = pressure;
  Sig.xx = Sig.zz = 0.0;
  Sig.xy = Sig.yx = 0.0;
  Sig.xz = Sig.yz = 0.0;
  Sig.yz = Sig.zy = 0.0;

  v.yy = 0.0; // free in fact
  v.xx = v.zz = 0.0;
  v.yx = 0.0;
  v.xz = v.zx = 0.0;
  v.yz = v.zy = 0.0;
  v.xy = 0.0; // will be driven by the servoFunction

  ServoFunction = [gammaDot](PBC3Dbox &box) -> void { box.Load.v.xy = gammaDot * box.Cell.h.yy; };
}

void Loading::VelocityControl(mat9r &V) {
  sprintf(StoredCommand, "VelocityControl %g %g %g   %g %g %g   %g %g %g", V.xx, V.xy, V.xz, V.yx, V.yy, V.yz, V.zx,
          V.zy, V.zz);
  Drive.reset(VelocityDriven);
  Sig.reset();
  v = V;
  ServoFunction = nullptr;
}

// This loading can be usefull for multiscale modeling (FEMxDEM or MPMxDEM).
void Loading::TransformationGradient(mat9r &h, mat9r &F, double duration) {
 std::cout<<"entering Loading::TransformationGradient"<<std::endl;
  sprintf(StoredCommand, "TransformationGradient %g %g %g %g %g %g %g %g %g",F.xx, F.xy, F.xz, F.yx, F.yy, F.yz, F.zx, F.zy, F.zz);
  Drive.reset(VelocityDriven);
  Sig.reset();
  std::cout<<"TG : reset done"<<std::endl;
  mat9r FmI = F;
  FmI.xx -= 1.0;
  FmI.yy -= 1.0;
  FmI.zz -= 1.0;
  std::cout<<"TG : F defined "<<std::endl;
  v = (1.0 / duration) * (FmI * h);
  std::cout<<"TG : v defined "<<std::endl;
  //ServoFunction = nullptr;
}

void Loading::LodeAnglePath(double pressure, double sigRate, double LodeAngle) {
  sprintf(StoredCommand, "LodeAnglePath %g %g %g", pressure, sigRate, LodeAngle);

  Drive.xx = Drive.yy = Drive.zz = ForceDriven;
  Drive.xy = Drive.yx = VelocityDriven;
  Drive.xz = Drive.zx = VelocityDriven;
  Drive.yz = Drive.zy = VelocityDriven;

  Sig.xx = Sig.yy = Sig.zz = pressure;
  Sig.xy = Sig.yx = 0.0;
  Sig.xz = Sig.yz = 0.0;
  Sig.yz = Sig.zy = 0.0;

  v.xx = v.yy = v.zz = 0.0; // free in fact (driven by the ServoFunction)
  v.xy = v.yx = 0.0;
  v.xz = v.zx = 0.0;
  v.yz = v.zy = 0.0;

  // Here LodeAngle expresses in degrees clockwise (should be in range[0° to 60°] )
  ServoFunction = [pressure, sigRate, LodeAngle](PBC3Dbox &box) -> void {
    double dSig = sigRate * box.t; // WARNING: initial time MUST be ZERO; dSig(t=0) = 0
    double LodeRadians = LodeAngle * M_PI / 180.0;
    double a = (3.0 * tan(LodeRadians) - sqrt(3.0)) / (2.0 * sqrt(3.0));
    double b = -(1.0 + a);
    box.Load.Sig.xx = pressure + dSig;
    box.Load.Sig.yy = pressure + (a * dSig);
    box.Load.Sig.zz = pressure + (b * dSig);
  };
}


void Loading::LodeAnglePathMix(double pressure, double velocity, double LodeAngle) {
  sprintf(StoredCommand, "LodeAnglePathMix %g %g %g", pressure, velocity, LodeAngle);

  Drive.yy = VelocityDriven;
  Drive.xx = Drive.zz = ForceDriven;
  Drive.xy = Drive.yx = ForceDriven;
  Drive.xz = Drive.zx = ForceDriven;
  Drive.yz = Drive.zy = ForceDriven;

  Sig.yy = 0.0;
  Sig.xx = Sig.zz = pressure; // initial stress state
  Sig.xy = Sig.yx = 0.0;
  Sig.xz = Sig.yz = 0.0;
  Sig.yz = Sig.zy = 0.0;

  v.yy = -velocity;
  v.xx = v.zz = 0.0; // free in fact (driven by the ServoFunction)
  v.xy = v.yx = 0.0;
  v.xz = v.zx = 0.0;
  v.yz = v.zy = 0.0;

  // Here LodeAngle expresses in degrees clockwise (should be in range[0° to 60°] )
  ServoFunction = [pressure, LodeAngle](PBC3Dbox &box) -> void {
    double Sigyy = box.Sig.yy;
    double LodeRadians = LodeAngle * M_PI / 180.0;
    double a = (3.0 * tan(LodeRadians) - sqrt(3.0)) / (2.0 * sqrt(3.0));
    double b = -(1.0 + a);
    double dSigyy = Sigyy - pressure;
    
    box.Load.Sig.xx = pressure + (b * dSigyy);
    box.Load.Sig.zz = pressure + (a * dSigyy);
  };
}


void Loading::Fixe() {
  sprintf(StoredCommand, "Fixe");
  Drive.reset(VelocityDriven);
  Sig.reset();
  v.reset();
  ServoFunction = nullptr;
}
