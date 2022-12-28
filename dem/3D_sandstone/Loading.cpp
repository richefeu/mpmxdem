#include "PBC3D.hpp"

Loading::Loading() {}

void Loading::TriaxialCompressionY(double pressure, double velocity) {
  snprintf(StoredCommand, 256, "TriaxialCompressionY %g %g", pressure, velocity);

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

  v.xx = v.zz = 0.0;  // free in fact
  v.yy = -velocity;
  v.xy = v.yx = 0.0;
  v.xz = v.zx = 0.0;
  v.yz = v.zy = 0.0;

  ServoFunction = nullptr;
}

void Loading::TriaxialCompressionZ(double pressure, double velocity) {
  snprintf(StoredCommand, 256, "TriaxialCompressionZ %g %g", pressure, velocity);

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

  v.xx = v.yy = 0.0;  // free in fact
  v.zz = -velocity;
  v.xy = v.yx = 0.0;
  v.xz = v.zx = 0.0;
  v.yz = v.zy = 0.0;

  ServoFunction = nullptr;
}

void Loading::BiaxialCompressionYPlaneStrainZ(double pressure, double velocity) {
  snprintf(StoredCommand, 256, "BiaxialCompressionYPlaneStrainZ %g %g", pressure, velocity);

  Drive.xx = ForceDriven;     // Pressure applied along x
  Drive.yy = VelocityDriven;  // Velocity imposed
  Drive.zz = VelocityDriven;  // Plane Strain condition
  Drive.xy = Drive.yx = VelocityDriven;
  Drive.xz = Drive.zx = VelocityDriven;
  Drive.yz = Drive.zy = VelocityDriven;

  Sig.xx = pressure;
  Sig.yy = 0.0;
  Sig.zz = 0.0;
  Sig.xy = Sig.yx = 0.0;
  Sig.xz = Sig.yz = 0.0;
  Sig.yz = Sig.zy = 0.0;

  v.xx = 0.0;  // free in fact
  v.yy = -velocity;
  v.zz = 0.0;
  v.xy = v.yx = 0.0;
  v.xz = v.zx = 0.0;
  v.yz = v.zy = 0.0;

  ServoFunction = nullptr;
}

void Loading::BiaxialCompressionZPlaneStrainX(double pressure, double velocity) {
  snprintf(StoredCommand, 256, "BiaxialCompressionZPlaneStrainX %g %g", pressure, velocity);

  Drive.yy = ForceDriven;     // Pressure applied along y
  Drive.xx = VelocityDriven;  // Velocity imposed
  Drive.zz = VelocityDriven;  // Plane Strain condition

  Drive.xy = Drive.yx = VelocityDriven;
  Drive.xz = Drive.zx = VelocityDriven;
  Drive.yz = Drive.zy = VelocityDriven;

  Sig.xx = 0.0;
  Sig.zz = 0.0;
  Sig.yy = pressure;
  Sig.xy = Sig.yx = 0.0;
  Sig.xz = Sig.yz = 0.0;
  Sig.yz = Sig.zy = 0.0;

  v.xx = 0.0;
  v.yy = 0.0;  // free in fact
  v.zz = -velocity;
  v.xy = v.yx = 0.0;
  v.xz = v.zx = 0.0;
  v.yz = v.zy = 0.0;

  ServoFunction = nullptr;
}

void Loading::BiaxialCompressionXPlaneStrainY(double pressure, double velocity) {
  snprintf(StoredCommand, 256, "BiaxialCompressionXPlaneStrainY %g %g", pressure, velocity);

  Drive.yy = ForceDriven;     // Pressure applied along y
  Drive.xx = VelocityDriven;  // Velocity imposed
  Drive.zz = VelocityDriven;  // Plane Strain condition

  Drive.xy = Drive.yx = VelocityDriven;
  Drive.xz = Drive.zx = VelocityDriven;
  Drive.yz = Drive.zy = VelocityDriven;

  Sig.xx = 0.0;
  Sig.yy = pressure;
  Sig.zz = 0.0;
  Sig.xy = Sig.yx = 0.0;
  Sig.xz = Sig.yz = 0.0;
  Sig.yz = Sig.zy = 0.0;

  v.zz = 0.0;
  v.yy = 0.0;  // free in fact
  v.xx = velocity;
  v.xy = v.yx = 0.0;
  v.xz = v.zx = 0.0;
  v.yz = v.zy = 0.0;

  ServoFunction = nullptr;
}

void Loading::IsostaticCompression(double pressure) {
  snprintf(StoredCommand, 256, "IsostaticCompression %g", pressure);

  Drive.xx = Drive.yy = Drive.zz = ForceDriven;
  Drive.xy = Drive.yx = VelocityDriven;
  Drive.xz = Drive.zx = VelocityDriven;
  Drive.yz = Drive.zy = VelocityDriven;

  Sig.xx = Sig.yy = Sig.zz = pressure;
  Sig.xy = Sig.yx = 0.0;
  Sig.xz = Sig.yz = 0.0;
  Sig.yz = Sig.zy = 0.0;

  v.xx = v.yy = v.zz = 0.0;  // free in fact
  v.xy = v.yx = 0.0;
  v.xz = v.zx = 0.0;
  v.yz = v.zy = 0.0;

  ServoFunction = nullptr;
}

void Loading::RigidRotationZ(double omega) {
  snprintf(StoredCommand, 256, "RigidRotationZ %g", omega);
  Drive.reset(VelocityDriven);
  Sig.reset();
  v.reset();
  ServoFunction = [omega](PBC3Dbox& box) -> void {
    double lx = sqrt(box.Cell.h.xx * box.Cell.h.xx + box.Cell.h.yx * box.Cell.h.yx + box.Cell.h.zx * box.Cell.h.zx);
    double ly = sqrt(box.Cell.h.xy * box.Cell.h.xy + box.Cell.h.yy * box.Cell.h.yy + box.Cell.h.zy * box.Cell.h.zy);
    double omegat = omega * box.t;  // t initial must be zero
    box.Load.v.xx = -lx * omega * sin(omegat);
    box.Load.v.yx = lx * omega * cos(omegat);

    box.Load.v.xy = -ly * omega * cos(omegat);
    box.Load.v.yy = -ly * omega * sin(omegat);
  };
}

void Loading::BiaxialCompressionY(double pxz, double py) {
  snprintf(StoredCommand, 256, "BiaxialCompressionY %g %g", pxz, py);

  Drive.xx = Drive.yy = Drive.zz = ForceDriven;
  Drive.xy = Drive.yx = VelocityDriven;
  Drive.xz = Drive.zx = VelocityDriven;
  Drive.yz = Drive.zy = VelocityDriven;

  Sig.xx = Sig.zz = pxz;
  Sig.yy = py;
  Sig.xy = Sig.yx = 0.0;
  Sig.xz = Sig.yz = 0.0;
  Sig.yz = Sig.zy = 0.0;

  v.xx = v.yy = v.zz = 0.0;  // free in fact
  v.xy = v.yx = 0.0;
  v.xz = v.zx = 0.0;
  v.yz = v.zy = 0.0;

  ServoFunction = nullptr;
}

// Shear in X direction while applying a pressure in Y direction
void Loading::SimpleShearXY(double pressure, double gammaDot) {
  snprintf(StoredCommand, 256, "SimpleShearXY %g %g", pressure, gammaDot);

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

  v.yy = 0.0;  // free in fact
  v.xx = v.zz = 0.0;
  v.yx = 0.0;
  v.xz = v.zx = 0.0;
  v.yz = v.zy = 0.0;
  v.xy = 0.0;  // will be driven by the servoFunction

  ServoFunction = [gammaDot](PBC3Dbox& box) -> void { box.Load.v.xy = gammaDot * box.Cell.h.yy; };
}

void Loading::VelocityControl(mat9r& V) {
  snprintf(StoredCommand, 256, "VelocityControl %g %g %g   %g %g %g   %g %g %g", V.xx, V.xy, V.xz, V.yx, V.yy, V.yz,
           V.zx, V.zy, V.zz);
  Drive.reset(VelocityDriven);
  Sig.reset();
  v = V;
  ServoFunction = nullptr;
}
void Loading::StrainControl(mat9r& F) {
  snprintf(StoredCommand, 256, "VelocityControl %g %g %g   %g %g %g   %g %g %g", F.xx, F.xy, F.xz, F.yx, F.yy, F.yz,
           F.zx, F.zy, F.zz);
  Drive.reset(VelocityDriven);
  Sig.reset();
  f = F;
  ServoFunction = nullptr;
}

// This loading can be usefull for multiscale modeling (FEMxDEM or MPMxDEM).
void Loading::TransformationGradient(mat9r& h, mat9r& F, double duration) {
  snprintf(StoredCommand, 256, "TransformationGradient %g %g %g %g %g %g %g %g %g", F.xx, F.xy, F.xz, F.yx, F.yy, F.yz,
           F.zx, F.zy, F.zz);
  Drive.reset(VelocityDriven);
  Sig.reset();
  mat9r FmI = F;
  FmI.xx -= 1.0;
  FmI.yy -= 1.0;
  FmI.zz -= 1.0;
  v = (1.0 / duration) * (FmI * h);
  ServoFunction = nullptr;
}

void Loading::LodeAnglePath(double pressure, double sigRate, double LodeAngle) {
  snprintf(StoredCommand, 256, "LodeAnglePath %g %g %g", pressure, sigRate, LodeAngle);

  Drive.xx = Drive.yy = Drive.zz = ForceDriven;
  Drive.xy = Drive.yx = VelocityDriven;
  Drive.xz = Drive.zx = VelocityDriven;
  Drive.yz = Drive.zy = VelocityDriven;

  Sig.xx = Sig.yy = Sig.zz = pressure;
  Sig.xy = Sig.yx = 0.0;
  Sig.xz = Sig.yz = 0.0;
  Sig.yz = Sig.zy = 0.0;

  v.xx = v.yy = v.zz = 0.0;  // free in fact (driven by the ServoFunction)
  v.xy = v.yx = 0.0;
  v.xz = v.zx = 0.0;
  v.yz = v.zy = 0.0;

  // Here LodeAngle expresses in degrees clockwise (should be in range[0° to 60°] )
  ServoFunction = [pressure, sigRate, LodeAngle](PBC3Dbox& box) -> void {
    double dSig = sigRate * box.t;  // WARNING: initial time MUST be ZERO; dSig(t=0) = 0
    double LodeRadians = LodeAngle * M_PI / 180.0;
    double a = (3.0 * tan(LodeRadians) - sqrt(3.0)) / (2.0 * sqrt(3.0));
    double b = -(1.0 + a);
    box.Load.Sig.xx = pressure + dSig;
    box.Load.Sig.yy = pressure + (a * dSig);
    box.Load.Sig.zz = pressure + (b * dSig);
  };
}

void Loading::LodeAnglePathMix(double pressure, double velocity, double LodeAngle) {
  snprintf(StoredCommand, 256, "LodeAnglePathMix %g %g %g", pressure, velocity, LodeAngle);

  Drive.yy = VelocityDriven;
  Drive.xx = Drive.zz = ForceDriven;
  Drive.xy = Drive.yx = ForceDriven;
  Drive.xz = Drive.zx = ForceDriven;
  Drive.yz = Drive.zy = ForceDriven;

  Sig.yy = 0.0;
  Sig.xx = Sig.zz = pressure;  // initial stress state
  Sig.xy = Sig.yx = 0.0;
  Sig.xz = Sig.yz = 0.0;
  Sig.yz = Sig.zy = 0.0;

  v.yy = -velocity;
  v.xx = v.zz = 0.0;  // free in fact (driven by the ServoFunction)
  v.xy = v.yx = 0.0;
  v.xz = v.zx = 0.0;
  v.yz = v.zy = 0.0;

  // Here LodeAngle expresses in degrees clockwise (should be in range[0° to 60°] )
  ServoFunction = [pressure, LodeAngle](PBC3Dbox& box) -> void {
    double Sigyy = box.Sig.yy;
    double LodeRadians = LodeAngle * M_PI / 180.0;
    double a = (3.0 * tan(LodeRadians) - sqrt(3.0)) / (2.0 * sqrt(3.0));
    double b = -(1.0 + a);
    double dSigyy = Sigyy - pressure;

    box.Load.Sig.xx = pressure + (b * dSigyy);
    box.Load.Sig.zz = pressure + (a * dSigyy);
  };
}

void Loading::AxisRotationZ(double E0, double omega, double Lx, double Ly, double iniTime) {
  snprintf(StoredCommand, 256, "AxisRotationZ %g %g", E0, omega);

  Drive.reset(VelocityDriven);
  Sig.reset();
  v.reset();

  ServoFunction = [E0, omega, Lx, Ly, iniTime](PBC3Dbox& box) -> void {
    double V0 = Lx * Ly;
    double a = box.Cell.h.xy / (2.0 * V0);
    double c = box.Cell.h.xx / (2.0 * V0);
    double d = box.Cell.h.xx / (Lx * Lx);
    double e = -box.Cell.h.yy / (Ly * Ly);
    double ff = -box.Cell.h.xy / (Ly * Ly);
    double g = box.Cell.h.yy;
    double h = box.Cell.h.xx;
    double det = a * ff * h - c * d * h + e * c * g;

#if 0

    double Co = 0.5 * E0 * omega * cos(omega * (box.t - iniTime));
    double Si = -E0 * omega * sin(omega * (box.t - iniTime));

    box.Load.v.xx = (f * g * Co - c * h * Si) / det;
    box.Load.v.yy = (-f * g * Co + c * g * Si) / det;
    box.Load.v.xy = ((e * g - d * h) * Co + a * h * Si) / det;
    box.Load.v.yx = 0.0;

#else
    // Cette version fait le calcul exacte de l'incrément de h : hinc
    // pour définir une vitesse vh telle que vh dt = hinc

    double intCo = (0.5 * E0 * (sin(omega * (iniTime - box.t)) - sin(omega * (iniTime - box.dt - box.t))));
    double intSi = E0 * (cos(omega * (iniTime - box.dt - box.t)) - cos(omega * (iniTime - box.t)));

    box.Load.v.xx = (ff * g * intCo - c * h * intSi) / (det * box.dt);
    box.Load.v.yy = (-ff * g * intCo + c * g * intSi) / (det * box.dt);
    box.Load.v.xy = ((e * g - d * h) * intCo + a * h * intSi) / (det * box.dt);
    box.Load.v.yx = 0.0;

#endif
  };
}

void Loading::Fixe() {
  snprintf(StoredCommand, 256, "Fixe");
  Drive.reset(VelocityDriven);
  Sig.reset();
  v.reset();
  ServoFunction = nullptr;
}
