// =========================================
// Periodic boundary conditions with spheres
// =========================================
// This code is used with lagamine for FEMxDEM (or MPMxDEM) multiscale modeling.
// It can also be used alone.

#include "PBC3D.hpp"

/// ----------------------------------------------------------------------------------------------------
/// ----------------------------------------------------------------------------------------------------
PBC3Dbox::PBC3Dbox() {
  // Some default values (actually, most of them will be (re-)set after)
  oldVersion = false;
  NLStrategy = 1;
  t = 0.0;
  tmax = 5.0;
  dt = 1e-6;
  interVerletC = 0.0;
  interVerlet = 0.01;
  interOutC = 0.0;
  interOut = 0.1;
  interConfC = 0.0;
  interConf = 0.25;
  density = 2700.0;
  kn = 0.0;
  kt = 0.0;
  kr = 0.0;
  dampRate = 0.95;
  mu = 0.8;
  mur = 0.0;
  fcoh = 0.0;
  fn0 = 0.0;
  ft0 = 0.0;
  mom0 = 0.0;
  dn0 = 0.0;
  dt0 = 0.0;
  drot0 = 0.0;
  powSurf = 2.0;
  iconf = 0;
  nbActiveInteractions = 0;
  nbBonds = 0;
  permamentGluer = 0;
  numericalDampingCoeff = 0.0;
  Kratio = 1.0;
  nbBondsini = 0;
  porosityini = 0;
  tensfailure = 0;
  fricfailure = 0;
  dVerlet = 1e-7;
  zetaMax = 1.0;
  zetaInter = 1.0;
  substractMeanVelocity = 1;
  limitHboxvelocity = 0;
  hboxLimitVel = 1e12;
  objectiveFriction = 1;
  rampRatio = 1.0;
  rampDuration = 0.0;
  continuumContact = 0;
  enableSwitch = 1;
  kineticStress = 0;

  modelSoftening = "trainee";
  setTraineeSoftening();
}

/// ----------------------------------------------------------------------------------------------------
/// @brief Print a banner related with the current code
/// ----------------------------------------------------------------------------------------------------
void PBC3Dbox::showBanner() {
  std::cout << "\nPBC3D (sandstone version), Periodic Boundary Conditions in 3D\n";
  std::cout << "<Vincent.Richefeu@3sr-grenoble.fr>\n\n";
}

/// ----------------------------------------------------------------------------------------------------
/// @brief open the files that will hold computation data
///        (macro-stress, cell config, macro-strain, static quality indices)
/// ----------------------------------------------------------------------------------------------------
void PBC3Dbox::initOutputFiles() {
  stressOut.open("stress.out.txt");
  cellOut.open("cell.out.txt");
  strainOut.open("strain.out.txt");
  resultantOut.open("resultant.out.txt");
}

/// ----------------------------------------------------------------------------------------------------
/// @brief Clear Particles and Interactions
/// ----------------------------------------------------------------------------------------------------
void PBC3Dbox::clearMemory() {
  Particles.clear();
  Interactions.clear();
}

/// ----------------------------------------------------------------------------------------------------
/// @brief Save the current configuration
/// @param[in] i File number. It will be named 'confx' where x is replaced by i
/// ----------------------------------------------------------------------------------------------------
void PBC3Dbox::saveConf(int i) {
  char fname[256];
  snprintf(fname, 256, "conf%d", i);
  saveConf(fname);
}

/// ----------------------------------------------------------------------------------------------------
/// ----------------------------------------------------------------------------------------------------
void PBC3Dbox::saveConf(const char* name) {
  std::ofstream conf(name);

  conf << "PBC3D 06-05-2021\n";  // format: progName version-date
  conf << "t " << t << '\n';
  conf << "tmax " << tmax << '\n';
  conf << "dt " << dt << '\n';
  conf << "interVerlet " << interVerlet << '\n';
  conf << "interOut " << interOut << '\n';
  conf << "interConf " << interConf << '\n';
  conf << "dVerlet " << dVerlet << '\n';
  conf << "NLStrategy " << NLStrategy << '\n';
  conf << "density " << density << '\n';
  conf << "kn " << kn << '\n';
  conf << "kt " << kt << '\n';
  conf << "kr " << kr << '\n';
  conf << "dampRate " << dampRate << '\n';
  conf << "mu " << mu << '\n';
  conf << "mur " << mur << '\n';
  conf << "fcoh " << fcoh << '\n';
  conf << "fn0 " << fn0 << '\n';
  conf << "ft0 " << ft0 << '\n';
  conf << "mom0 " << mom0 << '\n';
  // conf << "dn0 " << dn0 << '\n';
  // conf << "dt0 " << dt0 << '\n';
  // conf << "drot0 " << drot0 << '\n';
  conf << "powSurf " << powSurf << '\n';
  conf << "zetaMax " << zetaMax << '\n';
  conf << "zetaInter " << zetaInter << '\n';
  conf << "permamentGluer " << permamentGluer << '\n';
  conf << "continuumContact " << continuumContact << '\n';
  conf << "numericalDampingCoeff " << numericalDampingCoeff << '\n';
  conf << "Kratio " << Kratio << '\n';
  conf << "rampDuration " << rampDuration << '\n';
  conf << "iconf " << iconf << '\n';

  // From here, the written precision is increased to the maximum possible
  // (it is necessary because of the ASCII writting)
  conf << std::scientific << std::setprecision(std::numeric_limits<double>::digits10 + 1);

  conf << "h " << Cell.h << '\n';
  conf << "vh " << Cell.vh << '\n';
  conf << "ah " << Cell.ah << '\n';
  conf << "hmass " << Cell.mass << '\n';
  conf << "hvelGrad " << Cell.velGrad << '\n';
  conf << "hstrain " << Cell.strain << '\n';
  conf << "Sig " << Sig << '\n';
  if (limitHboxvelocity > 0) {
    conf << "limitHboxvelocity " << hboxLimitVel << '\n';
  }
  conf << "substractMeanVelocity " << substractMeanVelocity << '\n';
  conf << "objectiveFriction " << objectiveFriction << '\n';
  conf << "Load " << Load.StoredCommand << '\n';
  conf << "interVerletC " << interVerletC << '\n';
  conf << "interOutC " << interOutC << '\n';
  conf << "interConfC " << interConfC << '\n';
  conf << "nbBonds " << nbBonds << '\n';
  conf << "nbActiveInteractions " << nbActiveInteractions << '\n';
  conf << "nbBondsini " << nbBondsini << '\n';
  conf << "porosityini " << porosityini << '\n';
  conf << "tensfailure " << tensfailure << '\n';
  conf << "fricfailure " << fricfailure << '\n';
  if (enableSwitch == 0) {  // by default, switch is enabled
    conf << "enableSwitch " << enableSwitch << '\n';
  }
  conf << "kineticStress " << kineticStress << '\n';
  conf << "Particles " << Particles.size() << '\n';
  for (size_t i = 0; i < Particles.size(); i++) {
    conf << Particles[i].pos << ' ' << Particles[i].vel << ' ' << Particles[i].acc << ' ' << Particles[i].Q << ' '
         << Particles[i].vrot << ' ' << Particles[i].arot << ' ' << Particles[i].radius << ' ' << Particles[i].inertia
         << ' ' << Particles[i].mass << '\n';
  }
  conf << "Interactions " << nbActiveInteractions << '\n';
  if (oldVersion == true) {
    for (size_t i = 0; i < Interactions.size(); i++) {
      if (Interactions[i].state == noContactState) {
        continue;
      }
      conf << Interactions[i].i << ' ' << Interactions[i].j << ' ' << Interactions[i].gap0 << ' ' << Interactions[i].n
           << ' ' << Interactions[i].fn << ' ' << Interactions[i].fn_elas << ' ' << Interactions[i].fn_bond << ' '
           << Interactions[i].ft << ' ' << Interactions[i].ft_fric << ' ' << Interactions[i].ft_bond << ' '
           << Interactions[i].dt_fric << ' ' << Interactions[i].dt_bond << ' ' << Interactions[i].drot_bond << ' '
           << Interactions[i].mom << ' ' << Interactions[i].dampn << ' ' << Interactions[i].dampt << ' '
           << Interactions[i].state << ' ' << Interactions[i].D << '\n';
    }
  } else {
    for (size_t i = 0; i < Interactions.size(); i++) {
      if (Interactions[i].state == noContactState) {
        continue;
      }
      conf << Interactions[i].i << ' ' << Interactions[i].j << ' ' << Interactions[i].gap0 << ' ' << Interactions[i].n
           << ' ' << Interactions[i].fn << ' ' << Interactions[i].fn_elas << ' ' << Interactions[i].fn_bond << ' '
           << Interactions[i].ft << ' ' << Interactions[i].ft_fric << ' ' << Interactions[i].ft_bond << ' '
           << Interactions[i].dt_fric << ' ' << Interactions[i].dt_bond << ' ' << Interactions[i].drot_bond << ' '
           << Interactions[i].drot_fric << ' ' << Interactions[i].mom << ' ' << Interactions[i].mom_bond << ' '
           << Interactions[i].mom_fric << ' ' << Interactions[i].dampn << ' ' << Interactions[i].dampt << ' '
           << Interactions[i].state << ' ' << Interactions[i].D << '\n';
    }
  }
}

/// ----------------------------------------------------------------------------------------------------
/// @brief Load the configuration
/// @param[in]    name     Name of the file
/// ----------------------------------------------------------------------------------------------------
void PBC3Dbox::loadConf(const char* name) {
  double trash;
  // std::cout << "PBC3Dbox loading " << name << std::endl;
  std::ifstream conf(name);
  if (!conf.is_open()) {
    std::cerr << "@PBC3Dbox, Cannot read " << name << std::endl;
  }

  // Check header
  std::string prog;
  conf >> prog;
  if (prog != "PBC3D") {
    std::cerr << "@PBC3Dbox, This is not a file for PBC3D executable!" << std::endl;
  }
  std::string date;
  conf >> date;
  if (date != "06-05-2021") {
    std::cerr << "@PBC3Dbox, The version-date should be 06-05-2021!" << std::endl;
  }

  std::string token;
  conf >> token;
  while (conf.good()) {
    if (token[0] == '/' || token[0] == '#' || token[0] == '!') {
      getline(conf, token);  // ignore the rest of the current line
      conf >> token;         // next token
      continue;
    } else if (token == "t") {
      conf >> t;
    } else if (token == "tmax") {
      conf >> tmax;
    } else if (token == "dt") {
      conf >> dt;
      dt_2 = 0.5 * dt;
      dt2_2 = 0.5 * dt * dt;
    } else if (token == "interVerlet") {
      conf >> interVerlet;
    } else if (token == "interOut") {
      conf >> interOut;
    } else if (token == "interConf") {
      conf >> interConf;
    } else if (token == "dVerlet") {
      conf >> dVerlet;
    } else if (token == "NLStrategy") {
      conf >> NLStrategy;
    } else if (token == "density") {
      conf >> density;
    } else if (token == "kn") {
      conf >> kn;
    } else if (token == "kt") {
      conf >> kt;
    } else if (token == "kr") {
      conf >> kr;
    } else if (token == "dampRate") {
      conf >> dampRate;
    } else if (token == "mu") {
      conf >> mu;
    } else if (token == "mur") {
      conf >> mur;
    } else if (token == "fcoh") {
      conf >> fcoh;
    } else if (token == "fn0") {
      conf >> fn0;
      if (kn == 0.0) {
        std::cout << "ATTENTION ! kn = 0, (à definir avant fn0)\n";
      }
      dn0 = fn0 / kn;  // suppose that kn has been defined before
    } else if (token == "ft0") {
      conf >> ft0;
      if (kn == 0.0) {
        std::cout << "ATTENTION ! kt = 0, (à definir avant ft0)\n";
      }
      dt0 = ft0 / kt;  // suppose that kt has been defined before
    } else if (token == "mom0") {
      conf >> mom0;
      if (kn == 0.0) {
        std::cout << "ATTENTION ! kr = 0, (à definir avant mom0)\n";
      }
      drot0 = mom0 / kr;  // suppose that kr has been defined before
    } else if (token == "dn0") {
      conf >> dn0;
      if (kn == 0.0) {
        std::cout << "ATTENTION ! kn = 0, (à definir avant dn0)\n";
      }
      fn0 = kn * dn0;  // suppose that kn has been defined before
    } else if (token == "dt0") {
      conf >> dt0;
      if (kn == 0.0) {
        std::cout << "ATTENTION ! kt = 0, (à definir avant dt0)\n";
      }
      ft0 = kt * dt0;  // suppose that kt has been defined before
    } else if (token == "drot0") {
      conf >> drot0;
      if (kn == 0.0) {
        std::cout << "ATTENTION ! kr = 0, (à definir avant drot0)\n";
      }
      mom0 = kr * drot0;  // suppose that kr has been defined before
    } else if (token == "powSurf") {
      conf >> powSurf;
    } else if (token == "zetaMax") {
      conf >> zetaMax;
    } else if (token == "zetaInter") {
      conf >> zetaInter;
    } else if (token == "softening") {
      conf >> modelSoftening;
      if (modelSoftening == "linear") {
        setLinearSoftening();
      } else if (modelSoftening == "gate") {
        setGateSoftening();
      } else {
        setTraineeSoftening();
      }
    } else if (token == "permamentGluer") {
      conf >> permamentGluer;
    } else if (token == "continuumContact") {
      conf >> continuumContact;
    } else if (token == "rampDuration") {
      conf >> rampDuration;
    } else if (token == "numericalDampingCoeff") {
      conf >> numericalDampingCoeff;
    } else if (token == "Kratio") {
      conf >> Kratio;
    } else if (token == "iconf") {
      conf >> iconf;
    } else if (token == "h") {
      conf >> Cell.h;
    } else if (token == "vh") {
      conf >> Cell.vh;
    } else if (token == "ah") {
      conf >> Cell.ah;
    } else if (token == "hmass") {
      conf >> Cell.mass;
    } else if (token == "hvelGrad") {
      conf >> Cell.velGrad;
    } else if (token == "hstrain") {
      conf >> Cell.strain;
    } else if (token == "Sig") {
      conf >> Sig;
    } else if (token == "enableSwitch") {
      conf >> enableSwitch;
    } else if (token == "kineticStress") {
      conf >> kineticStress;
    } else if (token == "substractMeanVelocity") {
      conf >> substractMeanVelocity;
    } else if (token == "limitHboxvelocity") {
      conf >> hboxLimitVel;
      limitHboxvelocity = 1;
    } else if (token == "objectiveFriction") {
      conf >> objectiveFriction;
    } else if (token == "Load") {
      std::string command;
      conf >> command;
      if (command == "TriaxialCompressionY") {
        double pressure, velocity;
        conf >> pressure >> velocity;
        Load.TriaxialCompressionY(pressure, velocity);
      } else if (command == "TriaxialCompressionYGradual") {
        double pressure, velocity, ramp_duration;
        conf >> pressure >> velocity >> ramp_duration;
        Load.TriaxialCompressionYGradual(pressure, velocity, ramp_duration);
      } else if (command == "TriaxialCompressionZ") {
        double pressure, velocity;
        conf >> pressure >> velocity;
        Load.TriaxialCompressionZ(pressure, velocity);
      } else if (command == "TriaxialCompressionZGradual") {
        double pressure, velocity, ramp_duration;
        conf >> pressure >> velocity >> ramp_duration;
        Load.TriaxialCompressionZGradual(pressure, velocity, ramp_duration);
      } else if (command == "BiaxialCompressionYPlaneStrainZ") {
        double pressure, velocity;
        conf >> pressure >> velocity;
        Load.BiaxialCompressionYPlaneStrainZ(pressure, velocity);
      } else if (command == "BiaxialCompressionZPlaneStrainX") {
        double pressure, velocity;
        conf >> pressure >> velocity;
        Load.BiaxialCompressionZPlaneStrainX(pressure, velocity);
      } else if (command == "BiaxialCompressionXPlaneStrainY") {
        double pressure, velocity;
        conf >> pressure >> velocity;
        Load.BiaxialCompressionXPlaneStrainY(pressure, velocity);
      } else if (command == "BiaxialCompressionY") {
        double pxz, py;
        conf >> pxz >> py;
        Load.BiaxialCompressionY(pxz, py);
      } else if (command == "IsostaticCompression") {
        double pressure;
        conf >> pressure;
        Load.IsostaticCompression(pressure);
      } else if (command == "VelocityControl") {
        mat9r vh;
        conf >> vh;
        Load.VelocityControl(vh);
      } else if (command == "AxisRotationZ") {
        double E0, omega;
        conf >> E0 >> omega;
        // il faudrait calculer Lx et Ly à partir de h initial
        // donc les deux lignes suivantes sont ok uniquement si un fichier conf n'est pas repris
        double Lx = sqrt(Cell.h.xx * Cell.h.xx + Cell.h.yx * Cell.h.yx);
        double Ly = sqrt(Cell.h.xy * Cell.h.xy + Cell.h.yy * Cell.h.yy);
        Load.AxisRotationZ(E0, omega, Lx, Ly, t);
      } else if (command == "RigidRotationZ") {
        double omega;
        conf >> omega;
        Load.RigidRotationZ(omega);
      } else if (command == "StrainControl") {
        mat9r fh;
        conf >> fh;
        Load.StrainControl(fh);
      } else if (command == "TransformationGradient") {
        mat9r F;
        conf >> F.xx >> F.xy >> F.xz >> F.yx >> F.yy >> F.yz >> F.zx >> F.zy >> F.zz;
        Load.TransformationGradient(Cell.h, F, dt);
      } else if (command == "SimpleShearXY") {
        double pressure, gammaDot;
        conf >> pressure >> gammaDot;
        Load.SimpleShearXY(pressure, gammaDot);
      } else if (command == "LodeAnglePath") {
        double pressure, sigRate, LodeAngle;
        conf >> pressure >> sigRate >> LodeAngle;
        Load.LodeAnglePath(pressure, sigRate, LodeAngle);
        // MAYBE WE SHOULD IMPOSE t = 0 OR WARN IF IT'S  NOT THE CASE
      } else if (command == "LodeAnglePathMix") {
        double pressure, velocity, LodeAngle;
        conf >> pressure >> velocity >> LodeAngle;
        Load.LodeAnglePathMix(pressure, velocity, LodeAngle);
      } else if (command == "Fixe") {
        Load.Fixe();
      } else {
        std::cerr << "Unknown command for loading: " << command << std::endl;
      }
    } else if (token == "interVerletC") {
      conf >> trash;
    } else if (token == "interOutC") {
      conf >> trash;
    } else if (token == "interConfC") {
      conf >> trash;
    } else if (token == "nbBonds") {
      conf >> trash;
    } else if (token == "nbActiveInteractions") {
      conf >> trash;
    } else if (token == "nbBondsini") {
      conf >> trash;
    } else if (token == "porosityini") {
      conf >> trash;
    } else if (token == "tensfailure") {
      conf >> trash;
    } else if (token == "fricfailure") {
      conf >> trash;
    } else if (token == "Particles") {
      size_t nb;
      conf >> nb;
      Particles.clear();
      Particle P;
      for (size_t i = 0; i < nb; i++) {
        // remember that pos, vel and acc are expressed in reduced coordinates
        conf >> P.pos >> P.vel >> P.acc >> P.Q >> P.vrot >> P.arot >> P.radius >> P.inertia >> P.mass;
        Particles.push_back(P);
      }
    } else if (token == "noRollingFriction") {
      oldVersion = true;
    } else if (token == "Interactions") {
      size_t nb;
      conf >> nb;
      Interactions.clear();
      Interaction I;
      if (oldVersion == true) {
        for (size_t i = 0; i < nb; i++) {
          conf >> I.i >> I.j >> I.gap0 >> I.n >> I.fn >> I.fn_elas >> I.fn_bond >> I.ft >> I.ft_fric >> I.ft_bond >>
              I.dt_fric >> I.dt_bond >> I.drot_bond >> I.mom >> I.dampn >> I.dampt >> I.state >> I.D;
          Interactions.push_back(I);
        }
      } else {
        for (size_t i = 0; i < nb; i++) {
          conf >> I.i >> I.j >> I.gap0 >> I.n >> I.fn >> I.fn_elas >> I.fn_bond >> I.ft >> I.ft_fric >> I.ft_bond >>
              I.dt_fric >> I.dt_bond >> I.drot_bond >> I.drot_fric >> I.mom >> I.mom_bond >> I.mom_fric >> I.dampn >>
              I.dampt >> I.state >> I.D;
          Interactions.push_back(I);
        }
      }
    }
    /// ===========================================
    /// ============= PRE-PROCESSING ==============
    /// ===========================================
    else if (token == "ActivateBonds") {
      double epsiDist;
      conf >> epsiDist;
      ActivateBonds(epsiDist, bondedState);
    } else if (token == "nodamage") {
      zetaMax = 1.0;
    } else if (token == "ActivateDamageableBonds") {
      double epsiDist;
      conf >> epsiDist;
      ActivateBonds(epsiDist, bondedStateDam);
    } else if (token == "RemoveBondsRandomly") {
      double percentRemove;
      conf >> percentRemove;
      RemoveBonds(percentRemove, 0);
    } else if (token == "RemoveBondsSmallestPairs") {
      double percentRemove;
      conf >> percentRemove;
      RemoveBonds(percentRemove, 1);
    } else if (token == "RecomputeMassProperties") {
      Vsolid = 0.0;
      for (size_t i = 0; i < Particles.size(); i++) {
        double Vol = (4.0 / 3.0) * M_PI * Particles[i].radius * Particles[i].radius * Particles[i].radius;
        Vsolid += Vol;
        Particles[i].mass = Vol * density;
        Particles[i].inertia = (2.0 / 5.0) * Particles[i].mass * Particles[i].radius * Particles[i].radius;
      }
      Cell.mass = (Vsolid * density) / (double)(Particles.size());
    } else if (token == "RandomVelocities") {
      std::random_device rd;
      std::mt19937 gen(rd());

      double Vdev = 0.0;
      conf >> Vdev;
      std::normal_distribution<> vxDist{0.0, Vdev};
      std::normal_distribution<> vyDist{0.0, Vdev};
      std::normal_distribution<> vzDist{0.0, Vdev};

      mat9r hinv = Cell.h.get_inverse();
      for (size_t i = 0; i < Particles.size(); i++) {
        vec3r velReal(vxDist(gen), vyDist(gen), vzDist(gen));
        Particles[i].vel = hinv * velReal;
      }
    } else {
      std::cerr << "Unknown token: " << token << std::endl;
      exit(0);
    }

    conf >> token;
  }

  computeSampleData();
  accelerations();  // a fake time-increment that will compute missing thinks
}

/// ----------------------------------------------------------------------------------------------------
// This function computes some useful data related to the sample:
// Rmin, Rmax
// VelMin, velMax, VelMean
// Vsolid
// Vmin, Vmax, Vmean, etc.
//
// remark: it is called at the end of 'loadConf' and also in 'initLagamine'
//         or at the beginning of 'transform' for MPMxDEM
/// ----------------------------------------------------------------------------------------------------
void PBC3Dbox::computeSampleData() {
  // Particle related data
  if (!Particles.empty()) {
    Rmin = Rmax = Particles[0].radius;
    Rmean = 0.0;
    ReducedPartDistMean = 0.0;
    VelMin = VelMax = VelMean = VelVar = 0.0;
    AccMin = AccMax = AccMean = AccVar = 0.0;
    vec3r VectVelMean;
    vec3r VectAccMean;
    VectVelMean.reset();
    VectAccMean.reset();
    Vsolid = 0.0;
    Vmin = Vmax = Vmean = (4.0 / 3.0) * M_PI * Rmin * Rmin * Rmin;
    for (size_t i = 0; i < Particles.size(); i++) {
      double R = Particles[i].radius;
      double V = (4.0 / 3.0) * M_PI * R * R * R;
      Vsolid += V;

      Rmean += R;
      if (R > Rmax) {
        Rmax = R;
      }
      if (R < Rmin) {
        Rmin = R;
      }

      Vmean += V;
      if (V > Vmax) {
        Vmax = V;
      }
      if (V < Vmin) {
        Vmin = V;
      }

      vec3r Vel = Cell.vh * Particles[i].pos + Cell.h * Particles[i].vel;
      vec3r Acc = Cell.h * Particles[i].acc;
      VectVelMean += Vel;
      VectAccMean += Acc;
      double SqrVel = norm2(Vel);
      double SqrAcc = norm2(Acc);
      VelMean += SqrVel;
      AccMean += SqrAcc;
      if (SqrVel > VelMax) {
        VelMax = SqrVel;
      }
      if (SqrVel < VelMin) {
        VelMin = SqrVel;
      }
      if (SqrAcc > AccMax) {
        AccMax = SqrAcc;
      }
      if (SqrAcc < AccMin) {
        AccMin = SqrAcc;
      }
    }
    for (size_t k = 0; k < Interactions.size(); k++) {
      ReducedPartDistMean += (norm(Cell.h * (Particles[Interactions[k].i].pos - Particles[Interactions[k].j].pos)) +
                              Interactions[k].gap0) /
                             (Particles[Interactions[k].i].radius + Particles[Interactions[k].j].radius);
    }
    Rmean /= (double)Particles.size();
    Vmean /= (double)Particles.size();
    VectVelMean /= (double)Particles.size();
    VectAccMean /= (double)Particles.size();
    VelMean = sqrt(VelMean) / (double)Particles.size();
    VelMin = sqrt(VelMin);
    VelMax = sqrt(VelMax);
    AccMean = sqrt(AccMean) / (double)Particles.size();
    AccMin = sqrt(AccMin);
    AccMax = sqrt(AccMax);
    ReducedPartDistMean /= (double)Interactions.size();
    for (size_t i = 0; i < Particles.size(); i++) {
      vec3r Vel = Cell.vh * Particles[i].pos + Cell.h * Particles[i].vel;
      vec3r Acc = Cell.h * Particles[i].acc;
      VelVar += norm2(Vel - VectVelMean);
      AccVar += norm2(Acc - VectVelMean);
    }
    VelVar /= (double)Particles.size();
    AccVar /= (double)Particles.size();
  }

  // Interaction related data
  if (!Interactions.empty()) {
    FnMin = FnMax = Interactions[0].fn;
    FnMean = 0.0;
    nbActiveInteractions = 0;
    nbBonds = 0;
    for (size_t k = 0; k < Interactions.size(); k++) {
      if (Interactions[k].state == noContactState) {
        continue;
      }
      if (Interactions[k].state == bondedState) {
        nbBonds++;
      }
      if (Interactions[k].state == bondedStateDam) {
        nbBonds++;
      }
      nbActiveInteractions++;
      double Fn = Interactions[k].fn;
      FnMean += Fn;
      if (Fn > FnMax) {
        FnMax = Fn;
      }
      if (Fn < FnMin) {
        FnMin = Fn;
      }
    }
    if (nbActiveInteractions > 0) {
      FnMean /= (double)nbActiveInteractions;
    }
  }

  // weighting of stiffnesses
  w_bond = 1.0 / (Kratio + 1.0);
  w_particle = Kratio / (Kratio + 1.0);
}

/// ----------------------------------------------------------------------------------------------------
/// This method is callinged as a pre-processing command at the end of a conf-file.
/// It 'activates' each interaction by with normal distance lower than epsiDist
/// into a point of glue by changing its state
/// ----------------------------------------------------------------------------------------------------
void PBC3Dbox::ActivateBonds(double epsiDist, int state) {
  // In case the conf-file has no interactions, the neighbor list is updated
  updateNeighborList(dVerlet);

  for (size_t k = 0; k < Interactions.size(); k++) {
    size_t i = Interactions[k].i;
    size_t j = Interactions[k].j;

    vec3r sij = Particles[j].pos - Particles[i].pos;
    sij.x -= floor(sij.x + 0.5);
    sij.y -= floor(sij.y + 0.5);
    sij.z -= floor(sij.z + 0.5);
    vec3r branch = Cell.h * sij;

    double branchLen2 = norm2(branch);
    double sum = Particles[i].radius + Particles[j].radius + epsiDist;
    if (branchLen2 <= sum * sum) {
      // switch to a cemented/bonded link
      Interactions[k].state = state;
      Interactions[k].D = 0.0;

      double dn = sqrt(branchLen2) - (Particles[i].radius + Particles[j].radius);
      if (dn >= 0.0) {
        Interactions[k].gap0 = dn;
      } else {
        Interactions[k].gap0 = 0.0;
      }

      // compute the contact-point positions (relative to each particle)
      Interactions[k].n = branch;
      Interactions[k].n.normalize();
    }  // endif
  }  // end loop over interactions
}

/// ----------------------------------------------------------------------------------------------------
/// ----------------------------------------------------------------------------------------------------
void PBC3Dbox::RemoveBonds(double percentRemove, int StrategyId) {
  std::vector<size_t> indices;
  for (size_t i = 0; i < Interactions.size(); i++) {
    if (Interactions[i].state == bondedState || Interactions[i].state == bondedStateDam) {
      indices.push_back(i);
    }
  }
  if (StrategyId == 0) {
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(indices.begin(), indices.end(), g);
  } else if (StrategyId == 1) {
    sort(indices.begin(), indices.end(), [&](const size_t& a, const size_t& b) {
      double a_rmean = Particles[Interactions[a].i].radius + Particles[Interactions[a].j].radius;
      double b_rmean = Particles[Interactions[b].i].radius + Particles[Interactions[b].j].radius;
      return (a_rmean > b_rmean);
    });
  }
  size_t last = (size_t)floor((double)indices.size() * percentRemove * 0.01);
  for (size_t i = 0; i < last; i++) {
    size_t k = indices[i];
    if (Interactions[k].gap0 > 0.0) {
      Interactions[k].state = noContactState;
    } else {
      Interactions[k].state = contactState;
    }
  }
}

/// ----------------------------------------------------------------------------------------------------
// This is the most radical damping ever
/// ----------------------------------------------------------------------------------------------------
void PBC3Dbox::freeze() {
  for (size_t i = 0; i < Particles.size(); i++) {
    Particles[i].vel.reset();
    Particles[i].vrot.reset();
    Particles[i].acc.reset();
    Particles[i].arot.reset();
  }
}

/// ----------------------------------------------------------------------------------------------------
// Here is the framework used
//
//      z
//      |_ y
//    x/
//
/// ----------------------------------------------------------------------------------------------------
void PBC3Dbox::setSample() {
  Particle P;
  P.Q.reset();

  std::ofstream ans("answer-setSample.txt");

  char periodicShape = '1';
  while (true) {
    std::cout << "Shape of sample:\n    (1) Cube\n    (2) Rectangle\n> ";
    std::cin >> periodicShape;
    if (periodicShape == '1' || periodicShape == '2') {
      break;
    }
  }
  ans << (char)periodicShape << '\n';
  if (periodicShape == '1') {
    std::cout << "\nNumber of spheres on one side: ";
    std::cin >> ngz;
    ngx = ngy = ngz;
    ans << ngz << '\n';
  } else if (periodicShape == '2') {
    std::cout << "\nNumber of spheres on X side: ";
    std::cin >> ngx;
    ans << ngx << '\n';
    std::cout << "Number of spheres on Y side: ";
    std::cin >> ngy;
    ans << ngy << '\n';
    std::cout << "Number of spheres on Z side: ";
    std::cin >> ngz;
    ans << ngz << '\n';
  }
  ngw = (ngx > ngy ? ngx : (ngy > ngz ? ngy : ngz));

  double stepX = 1.0 / (2.0 * ngx);  // in the range [0, 1]
  double stepY = 1.0 / (2.0 * ngy);  // in the range [0, 1]
  double stepZ = 1.0 / (2.0 * ngz);  // in the range [0, 1]

  double radius = 1e-3;
  std::cout << "Maximum (larger) radius: ";
  std::cin >> radius;
  ans << radius << '\n';
  double deltaR = 0.2e-3;
  std::cout << "Range of radii (Delta R): ";
  std::cin >> deltaR;
  ans << deltaR << '\n';

  dVerlet = 2.5 * radius;

  density = 2700.0;
  std::cout << "Mass density of the particles: ";
  std::cin >> density;
  ans << density << '\n';

  char strategy = '1';
  while (true) {
    std::cout << "Packing strategy:\n    (1) grid\n    (2) Poisson sampling\n> ";
    std::cin >> strategy;
    if (strategy == '1' || strategy == '2') {
      break;
    }
  }
  ans << (char)strategy << '\n';

  if (strategy == '1') {
    double cellSizeX = 2.0 * ngx * radius;
    double cellSizeY = 2.0 * ngy * radius;
    double cellSizeZ = 2.0 * ngz * radius;
    Cell.Define(cellSizeX, 0.0, 0.0, 0.0, cellSizeY, 0.0, 0.0, 0.0, cellSizeZ);
    Vsolid = 0.0;
    double Vol;
    Particles.clear();
    for (int iz = 0; iz < ngz; iz++) {
      for (int iy = 0; iy < ngy; iy++) {
        for (int ix = 0; ix < ngx; ix++) {
          P.radius = radius - deltaR * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX));
          Vol = (4.0 / 3.0) * M_PI * P.radius * P.radius * P.radius;
          Vsolid += Vol;
          P.mass = Vol * density;
          P.inertia = (2.0 / 5.0) * P.mass * P.radius * P.radius;

          // reduced positions
          P.pos.x = stepX * (1.0 + 2.0 * ix);
          P.pos.y = stepY * (1.0 + 2.0 * iy);
          P.pos.z = stepZ * (1.0 + 2.0 * iz);
          Particles.push_back(P);
        }
      }
    }
    // To define
  } else if (strategy == '2') {
    double cellSize = 1.0;
    std::cout << "Cubic cell size: ";
    std::cin >> cellSize;
    ans << cellSize << '\n';

    unsigned int seedPerso = 0;
    std::cout << "seed? (0 for time-seeded): ";
    std::cin >> seedPerso;
    ans << seedPerso << '\n';

    Cell.Define(cellSize, 0.0, 0.0, 0.0, cellSize, 0.0, 0.0, 0.0, cellSize);

    GeoPack3D packing(radius - deltaR,  // rmin
                      radius,           // rmax
                      200,              // no of trials
                      0.0, cellSize, 0.0, cellSize, 0.0, cellSize, cellSize - (2.0 * ngw * radius),
                      ngx * ngy * ngz  // The periodic cell
    );

    if (seedPerso > 0) {
      srand(seedPerso);
    } else {
      packing.seedTime();
    }

    // MORE PARAMETRIZATION HERE ........ TODO
    packing.execPeriodic();

    Vsolid = 0.0;
    double Vol;
    mat9r hinv = Cell.h.get_inverse();
    Particles.clear();
    for (size_t i = 0; i < packing.sample.size(); i++) {
      P.radius = packing.sample[i].r;
      Vol = (4.0 / 3.0) * M_PI * P.radius * P.radius * P.radius;
      Vsolid += Vol;
      P.mass = Vol * density;
      P.inertia = (2.0 / 5.0) * P.mass * P.radius * P.radius;

      // reduced positions
      vec3r s(packing.sample[i].x, packing.sample[i].y, packing.sample[i].z);
      P.pos = hinv * s;
      Particles.push_back(P);
    }
  }

  Cell.mass = 1.0;
  std::cout << "Mass of the periodic cell?  Set it as (please choose 1):" << std::endl;
  std::cout << "   (1) mean mass of a single particle" << std::endl;
  std::cout << "   (2) approximative mass of a one-particle-depth slice" << std::endl;
  std::cout << "   (3) total mass" << std::endl;
  char repMass = '1';
  while (true) {
    std::cout << "choice: ";
    std::cin >> repMass;
    if (repMass == '1' || repMass == '2' || repMass == '3') {
      break;
    }
  }
  ans << (char)repMass << '\n';
  if (repMass == '1') {
    Cell.mass = (Vsolid * density) / (double)(ngx * ngy * ngz);
  } else if (repMass == '2') {
    Cell.mass = (Vsolid * density) / (double)(ngw);  // Taking the slightest slice, like a bread loaf?
  } else if (repMass == '3') {
    Cell.mass = (Vsolid * density);
  }

  // Set ramdom velocities to particles
  double vmax = 0.1;
  std::cout << "Norm of the randomly oriented velocities: ";
  std::cin >> vmax;
  ans << vmax << '\n';
  double vmaxReduced = vmax / (2.0 * ngw * radius);
  for (size_t i = 0; i < Particles.size(); i++) {
    Particles[i].vel.x = vmaxReduced * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 0.5);
    Particles[i].vel.y = vmaxReduced * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 0.5);
    Particles[i].vel.z = vmaxReduced * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 0.5);
  }

  // ===== Parametres de simu =====

  double press = 1000.0;
  std::cout << "Isostatic confinement pressure: ";
  std::cin >> press;
  ans << press << '\n';
  Load.IsostaticCompression(press);

  kn = 1.0e4;
  while (true) {
    char rep = 'o';
    std::cout << "Normal stiffness, kn: ";
    std::cin >> kn;
    ans << kn << '\n';
    std::cout << "which corresponds to kappa = kn/(ap) = " << kn / (2.0 * radius * press) << std::endl;
    std::cout << "Satisfied ? (y/n): ";
    std::cin >> rep;
    ans << (char)rep << '\n';
    if (rep == 'o' || rep == 'O' || rep == 'y' || rep == 'Y') {
      break;
    }
  }

  double ktkn = 1.0;
  std::cout << "Ratio kt/kn: ";
  std::cin >> ktkn;
  ans << ktkn << '\n';
  kt = ktkn * kn;

  dampRate = 0.95;
  std::cout << "Viscuous damping rate: ";
  std::cin >> dampRate;
  ans << dampRate << '\n';

  mu = 0.8;
  std::cout << "Friction coefficient: ";
  std::cin >> mu;
  ans << mu << '\n';

  dt = 1e-6;
  while (true) {
    char rep = 'o';
    std::cout << "Time step, dt: ";
    std::cin >> dt;
    ans << dt << '\n';
    double m_max = (4.0 / 3.0) * M_PI * radius * radius * radius * density;
    double m_min = (4.0 / 3.0) * M_PI * (radius - deltaR) * (radius - deltaR) * (radius - deltaR) * density;
    std::cout << "which corresponds to dt_crit/dt in [ " << (M_PI * sqrt(m_min / kn)) / dt << " , "
              << (M_PI * sqrt(m_max / kn)) / dt << " ]" << std::endl;
    std::cout << "Satisfied? (y/n): ";
    std::cin >> rep;
    ans << (char)rep << '\n';
    if (rep == 'o' || rep == 'O' || rep == 'y' || rep == 'Y') {
      break;
    }
  }

  dt_2 = 0.5 * dt;
  dt2_2 = 0.5 * dt * dt;

  tmax = 5.0;
  std::cout << "Simulated duration: ";
  std::cin >> tmax;
  ans << tmax << '\n';

  t = 0.0;
  nbActiveInteractions = 0;

  // Reset counters
  iconf = 0;
  interVerletC = 0.0;
  interOutC = 0.0;
  interConfC = 0.0;

  // Other parameters
  interVerlet = 10 * dt;
  interOut = 100 * dt;
  interConf = 100 * dt;

  std::cout << "Other parameters have been set to given values.\n"
            << "You can change it in the generated file.\n";
  std::cout << "interVerlet " << interVerlet << '\n';
  std::cout << "interOut " << interOut << '\n';
  std::cout << "interConf " << interConf << '\n';

  return;
}

/// ----------------------------------------------------------------------------------------------------
/// @brief Computes a single step with the velocity-Verlet algorithm
/// ----------------------------------------------------------------------------------------------------
void PBC3Dbox::velocityVerletStep() {
  START_TIMER("velocityVerletStep");

  if (Load.ServoFunction != nullptr) {
    Load.ServoFunction(*this);
  }

  for (size_t i = 0; i < Particles.size(); i++) {
    Particles[i].pos += dt * Particles[i].vel + dt2_2 * Particles[i].acc;
    Particles[i].vel += dt_2 * Particles[i].acc;

    // Periodicity in position, i.e. move the outgoing particles in the orther side
    // Vincent: Gael conviced me this is ok and absolutely mandatory, so even if the flag enableSwitch
    //          exists, it should always be activated (enableSwitch = 1)
    if (enableSwitch > 0) {
      bool recompute_velocity = false;
      for (size_t c = 0; c < 3; c++) {
        while (Particles[i].pos[c] < 0.0) {
          Particles[i].pos[c] += 1.0;
        }
        while (Particles[i].pos[c] > 1.0) {
          Particles[i].pos[c] -= 1.0;
        }
      }
    }

    // Rotation: Q = Q + (dQ/dt) * dt + (d2Q/dt2) * dt2/2
    // It reads like this with quaternions
    double omega2_2 = 0.5 * Particles[i].vrot * Particles[i].vrot;
    Particles[i].Q += quat(
        0.5 * (dt * (Particles[i].Q.s * Particles[i].vrot + cross(Particles[i].vrot, Particles[i].Q.v)) +
               dt2_2 * (Particles[i].Q.s * Particles[i].arot + cross(Particles[i].arot, Particles[i].Q.v) -
                        omega2_2 * Particles[i].Q.v)),
        -0.5 * (Particles[i].vrot * Particles[i].Q.v * dt + (Particles[i].arot * Particles[i].Q.v + omega2_2) * dt2_2));
    Particles[i].Q.normalize();

    Particles[i].vrot += dt_2 * Particles[i].arot;
  }

  for (size_t c = 0; c < 9; c++) {  // loop over components
    if (Load.Drive[c] == ForceDriven) {
      Cell.h[c] += dt * Cell.vh[c] + dt2_2 * Cell.ah[c];
      Cell.vh[c] += dt_2 * Cell.ah[c];
    } else {
      Cell.h[c] += dt * Load.v[c];
      Cell.vh[c] = Load.v[c];
      Cell.ah[c] = 0.0;
    }
  }

  accelerations();

  vec3r vmean;
  for (size_t i = 0; i < Particles.size(); i++) {
    Particles[i].vel += dt_2 * Particles[i].acc;
    vmean += Particles[i].vel;
    Particles[i].vrot += dt_2 * Particles[i].arot;
  }

  if (substractMeanVelocity > 0) {
    vmean /= (double)(Particles.size());
    for (size_t i = 0; i < Particles.size(); i++) {
      Particles[i].vel -= vmean;
    }
  }

  // velocity rampe
  if (rampDuration - t > 0.0) {
    rampRatio = t / rampDuration;
  } else {
    rampRatio = 1.0;
  }

  for (size_t c = 0; c < 9; c++) {
    if (Load.Drive[c] == ForceDriven) {
      Cell.vh[c] += rampRatio * dt_2 * Cell.ah[c];
      if (limitHboxvelocity > 0 && std::fabs(Cell.vh[c]) > hboxLimitVel) {
        Cell.vh[c] = std::copysign(hboxLimitVel, Cell.vh[c]);
      }
    }
  }

  Cell.update(dt);
}

/// ----------------------------------------------------------------------------------------------------
/// @brief Print information about the running computation and current state of the sample
/// ----------------------------------------------------------------------------------------------------
void PBC3Dbox::printScreen(double elapsedTime) {
  std::cout << "+\n";
  std::cout << "|  iconf = " << iconf << ", Time = " << t << '\n';
  std::cout << "|  Elapsed time since last configuration: " << elapsedTime << " seconds" << '\n';

  std::cout << "|  Stress: " << __FORMATED(3, 20, Sig.xx) << ' ' << __FORMATED(3, 20, Sig.xy) << ' '
            << __FORMATED(3, 20, Sig.xz) << '\n';
  std::cout << "|          " << __FORMATED(3, 20, Sig.yx) << ' ' << __FORMATED(3, 20, Sig.yy) << ' '
            << __FORMATED(3, 20, Sig.yz) << '\n';
  std::cout << "|          " << __FORMATED(3, 20, Sig.zx) << ' ' << __FORMATED(3, 20, Sig.zy) << ' '
            << __FORMATED(3, 20, Sig.zz) << '\n';

  std::cout << "|  Cell:   " << __FORMATED(6, 20, Cell.h.xx) << ' ' << __FORMATED(6, 20, Cell.h.xy) << ' '
            << __FORMATED(6, 20, Cell.h.xz) << '\n';
  std::cout << "|          " << __FORMATED(6, 20, Cell.h.yx) << ' ' << __FORMATED(6, 20, Cell.h.yy) << ' '
            << __FORMATED(6, 20, Cell.h.yz) << '\n';
  std::cout << "|          " << __FORMATED(6, 20, Cell.h.zx) << ' ' << __FORMATED(6, 20, Cell.h.zy) << ' '
            << __FORMATED(6, 20, Cell.h.zz) << '\n';

  double Vcell = fabs(Cell.h.det());
  std::cout << "|  Solid fraction: " << Vsolid / Vcell << '\n';
  std::cout << "|  dt_crit/dt in range [" << (M_PI * sqrt(Vmin * density / kn)) / dt << ", "
            << (M_PI * sqrt(Vmax * density / kn)) / dt << "]\n";
  double ResMean, Res0Mean, fnMin, fnMean;
  staticQualityData(&ResMean, &Res0Mean, &fnMin, &fnMean);
  std::cout << "|  Mean resultant: " << ResMean << ", Mean resultant (without ratlers): " << Res0Mean << '\n';
  std::cout << "|  Min contact normal force:  " << fnMin << ", Mean contact normal force:  " << fnMean << '\n';
  std::cout << "|  Res0Mean/fnMean: " << Res0Mean / fnMean << '\n';

  double vf = 0.0;
  double Kin = 0.0;
  double sqVel;
  vec3r vel;
  for (size_t i = 0; i < Particles.size(); i++) {
    vel = Cell.vh * Particles[i].pos + Cell.h * Particles[i].vel;
    sqVel = vel * vel;
    vf = std::max(vf, sqVel);
    Kin += Particles[i].mass * sqVel;
  }
  vf = sqrt(vf) * interVerlet;
  Kin *= 0.5;

  std::cout << "|  Kin energy (translations) : " << Kin << '\n';
  std::cout << "|  Estimated free flight distance between Neighbor updates: " << 1.0e6 * vf << "e-6\n";
  std::cout << "|  Verlet distance: " << dVerlet << ", Rmin: " << Rmin << '\n';

  std::cout << "+\n\n";
  std::cout << std::flush;
}

/// ----------------------------------------------------------------------------------------------------
/// @brief The main loop of time-integration
/// ----------------------------------------------------------------------------------------------------
void PBC3Dbox::integrate() {
  START_TIMER("integrate");
  // (re)-compute some constants in case they were not yet set
  dt_2 = 0.5 * dt;
  dt2_2 = 0.5 * dt * dt;
  accelerations();

  dataOutput();

  char fname[256];
  snprintf(fname, 256, "conf%d", iconf);
  if (!fileTool::fileExists(fname)) {
    saveConf(iconf);
  }

  double previousTime = (double)std::clock() / (double)CLOCKS_PER_SEC;
  while (t < tmax) {

    t += dt;
    interConfC += dt;
    interOutC += dt;
    interVerletC += dt;

    velocityVerletStep();

    if (interConfC >= interConf - dt_2) {
      iconf++;

      double currentTime = (double)std::clock() / (double)CLOCKS_PER_SEC;
      printScreen(currentTime - previousTime);
      previousTime = currentTime;

      saveConf(iconf);
      interConfC = 0.0;
    }

    if (interVerletC >= interVerlet - dt_2) {
      updateNeighborList(dVerlet);
      interVerletC = 0.0;
    }

    if (interOutC >= interOut - dt_2) {
      dataOutput();
      interOutC = 0.0;
    }
  }

  return;
}

/// ----------------------------------------------------------------------------------------------------
/// @brief Save data in output files
/// ----------------------------------------------------------------------------------------------------
void PBC3Dbox::dataOutput() {
  // Cell
  cellOut << t << ' ' << Cell.h << ' ' << Cell.vh << std::endl;

  // Stress
  stressOut << t << ' ' << Sig << std::endl;

  // Strain
  strainOut << t << ' ' << Cell.strain << std::endl;

  // Other things
  double ResMean, Res0Mean, fnMin, fnMean;
  double Vcell = fabs(Cell.h.det());

  double Kin = 0.0;
  vec3r vel;
  for (size_t i = 0; i < Particles.size(); i++) {
    vel = Cell.vh * Particles[i].pos + Cell.h * Particles[i].vel;
    Kin += Particles[i].mass * vel * vel;
  }
  Kin *= 0.5;

  staticQualityData(&ResMean, &Res0Mean, &fnMin, &fnMean);
  resultantOut << t << ' ' << ResMean << ' ' << Res0Mean << ' ' << fnMin << ' ' << fnMean << ' ' << nbBonds << ' '
               << tensfailure << ' ' << fricfailure << ' ' << Vcell << ' ' << VelMax << ' ' << VelMin << ' ' << VelMean
               << ' ' << VelVar << ' ' << ReducedPartDistMean << ' ' << Kin << std::endl;
}

/// ----------------------------------------------------------------------------------------------------
/// ----------------------------------------------------------------------------------------------------
void PBC3Dbox::updateNeighborList(double dmax) {
  switch (NLStrategy) {
    case 0: {
      updateNeighborList_brutForce(dmax);
    } break;
    case 1: {
      updateNeighborList_linkCells(dmax);
    } break;
    default:
      updateNeighborList_brutForce(dmax);
  }
}

/// ----------------------------------------------------------------------------------------------------
/// @brief  Update the neighbor list (that is the list of 'active' and 'non-active' interactions)
/// @param[in] dmax Maximum distance for adding an Interaction in the neighbor list
/// ----------------------------------------------------------------------------------------------------
void PBC3Dbox::updateNeighborList_linkCells(double dmax) {
  START_TIMER("updateNeighborList");

  // store ft because the list will be cleared before being rebuilt
  std::vector<Interaction> Ibak;
  for (size_t k = 0; k < Interactions.size(); k++) {
    Ibak.push_back(Interactions[k]);
  }

  // periodic linkCells
  Interactions.clear();

  size_t numPoints = Particles.size();
  std::vector<IdPoint> reducedPosPoints;
  for (size_t i = 0; i < numPoints; ++i) {
    reducedPosPoints.push_back(IdPoint(i, Particles[i].pos.x, Particles[i].pos.y, Particles[i].pos.z));
  }

  // Determine the gridSize based on the density of points
  // WARNING: this is an empirical recipes (one day, we will make something more robust)
  double optimumDistance = 1.8 * std::pow(1.0 / (double)numPoints, 1.0 / 3.0);
  size_t optimumGridSize = std::max<size_t>(static_cast<size_t>(std::floor(1.0 / optimumDistance)), 3);

  PeriodicNearestNeighbors perioNN(reducedPosPoints, optimumGridSize);
  std::vector<std::vector<size_t>> neighbors = perioNN.getNeighbors(/*optimumDistance*/);

  for (size_t i = 0; i < neighbors.size(); ++i) {
    for (size_t j : neighbors[i]) {

      vec3r sij = Particles[j].pos - Particles[i].pos;
      for (size_t c = 0; c < 3; c++) {
        sij[c] -= floor(sij[c] + 0.5);
      }
      vec3r branch = Cell.h * sij;

      double sum = dmax + Particles[i].radius + Particles[j].radius;
      if (norm2(branch) <= sum * sum) {
        double m = (Particles[i].mass * Particles[j].mass) / (Particles[i].mass + Particles[j].mass);
        double Dampn = dampRate * 2.0 * sqrt(kn * m);
        double Dampt = dampRate * 2.0 * sqrt(kt * m);
        Interactions.push_back(Interaction(i, j, Dampn, Dampt));
      }
    }
  }

  // no need to sort, it is already done
  // lexicographic sort of Interactions
  // std::sort(Interactions.begin(), Interactions.end());

  // retrieve previous contacts or bonds
  size_t k, kold = 0;
  for (k = 0; k < Interactions.size(); ++k) {
    while (kold < Ibak.size() && Ibak[kold].i < Interactions[k].i) {
      ++kold;
    }
    if (kold == Ibak.size()) {
      break;
    }

    while (kold < Ibak.size() && Ibak[kold].i == Interactions[k].i && Ibak[kold].j < Interactions[k].j) {
      ++kold;
    }
    if (kold == Ibak.size()) {
      break;
    }

    if (Ibak[kold].i == Interactions[k].i && Ibak[kold].j == Interactions[k].j) {
      Interactions[k] = Ibak[kold];
      ++kold;
    }
  }
}

/// ----------------------------------------------------------------------------------------------------
/// @brief  Update the neighbor list (that is the list of 'active' and 'non-active' interactions)
/// @param[in] dmax Maximum distance for adding an Interaction in the neighbor list
/// ----------------------------------------------------------------------------------------------------
void PBC3Dbox::updateNeighborList_brutForce(double dmax) {
  START_TIMER("updateNeighborList_brutForce");

  // store ft because the list will be cleared before being rebuilt
  std::vector<Interaction> Ibak;
  Interaction I;
  for (size_t k = 0; k < Interactions.size(); k++) {
    I = Interactions[k];
    Ibak.push_back(I);
  }

  // now rebuild the list
  Interactions.clear();
  for (size_t i = 0; i < Particles.size(); i++) {
    for (size_t j = i + 1; j < Particles.size(); j++) {

      vec3r sij = Particles[j].pos - Particles[i].pos;
      for (size_t c = 0; c < 3; c++) {
        sij[c] -= floor(sij[c] + 0.5);
      }
      vec3r branch = Cell.h * sij;

      double sum = dmax + Particles[i].radius + Particles[j].radius;
      if (norm2(branch) <= sum * sum) {
        double m = (Particles[i].mass * Particles[j].mass) / (Particles[i].mass + Particles[j].mass);
        double Dampn = dampRate * 2.0 * sqrt(kn * m);
        double Dampt = dampRate * 2.0 * sqrt(kt * m);
        Interactions.push_back(Interaction(i, j, Dampn, Dampt));
      }
    }
  }

  // retrieve previous contacts or bonds
  size_t k, kold = 0;
  for (k = 0; k < Interactions.size(); ++k) {
    while (kold < Ibak.size() && Ibak[kold].i < Interactions[k].i) {
      ++kold;
    }
    if (kold == Ibak.size()) {
      break;
    }

    while (kold < Ibak.size() && Ibak[kold].i == Interactions[k].i && Ibak[kold].j < Interactions[k].j) {
      ++kold;
    }
    if (kold == Ibak.size()) {
      break;
    }

    if (Ibak[kold].i == Interactions[k].i && Ibak[kold].j == Interactions[k].j) {
      Interactions[k] = Ibak[kold];
      ++kold;
    }
  }
}

/// ----------------------------------------------------------------------------------------------------
/// @brief Compute acceleration of the particles and of the periodic-cell.
/// ----------------------------------------------------------------------------------------------------
void PBC3Dbox::accelerations() {
  START_TIMER("accelerations");

  // Set forces and moments to zero
  for (size_t i = 0; i < Particles.size(); i++) {
    Particles[i].force.reset();
    Particles[i].moment.reset();
  }
  Sig.reset();  // reset internal stress

  nbActiveInteractions = 0;
  nbBonds = 0;
  tensfailure = 0;
  fricfailure = 0;

  computeForcesAndMoments();
  if (kineticStress > 0) {
    addKineticStress();
  }

  // Cundall damping for particles
  if (numericalDampingCoeff > 0.0) {
    double factor = 0.0;
    double factorMinus = 1.0 - numericalDampingCoeff;
    double factorPlus = 1.0 + numericalDampingCoeff;

    for (size_t i = 0; i < Particles.size(); i++) {
      factor = (Particles[i].force.x * Particles[i].vel.x > 0.0) ? factorMinus : factorPlus;
      Particles[i].force.x *= factor;
      factor = (Particles[i].force.y * Particles[i].vel.y > 0.0) ? factorMinus : factorPlus;
      Particles[i].force.y *= factor;
      factor = (Particles[i].force.z * Particles[i].vel.z > 0.0) ? factorMinus : factorPlus;
      Particles[i].force.z *= factor;
    }
  }

  double Vcell = fabs(Cell.h.det());
  Sig *= (1.0 / Vcell);

  // Compute inverse of the matrix h
  mat9r hinv = Cell.h.get_inverse();

  // Acceleration of the periodic cell
  mat9r Vhinv = Vcell * hinv;
  mat9r deltaSig = Sig - Load.Sig;
  double invMass = 1.0 / Cell.mass;

  for (size_t row = 0; row < 3; row++) {
    for (size_t col = 0; col < 3; col++) {
      size_t c = 3 * row + col;
      if (Load.Drive[c] == ForceDriven) {
        Cell.ah[c] = 0.0;
        for (size_t s = 0; s < 3; s++) {
          Cell.ah[c] += Vhinv[3 * row + s] * deltaSig[3 * s + col];
        }
        Cell.ah[c] *= invMass;
      }
    }
  }

  // Cundall damping for the periodic cell
  if (numericalDampingCoeff > 0.0) {
    double factor = 0.0;
    double factorMinus = 1.0 - numericalDampingCoeff;
    double factorPlus = 1.0 + numericalDampingCoeff;

    for (size_t row = 0; row < 3; row++) {
      for (size_t col = 0; col < 3; col++) {
        size_t c = 3 * row + col;
        if (Load.Drive[c] == ForceDriven) {
          factor = (Cell.ah[c] * Cell.vh[c] > 0.0) ? factorMinus : factorPlus;
          Cell.ah[c] *= factor;
        }
      }
    }
  }

  // Finally compute the accelerations (translation and rotation) of the particles
  for (size_t i = 0; i < Particles.size(); i++) {
    vec3r acc = Particles[i].force / Particles[i].mass;

#if 1
    // =====================================================
    // The following 2 lines can be removed.
    // In fact, mathematic relations say we should use them
    // but in practice their presence makes no difference
    // in the case of quasistatic strainning (ah and vh <<< 1)
    // =====================================================
    //acc -= Cell.ah * Particles[i].pos;
    acc -= 2.0 * Cell.vh * Particles[i].vel;
#endif

    Particles[i].acc = hinv * acc;

    Particles[i].arot = Particles[i].moment / Particles[i].inertia;  // It's ok for spheres
  }
}


/// ----------------------------------------------------------------------------------------------------
/// ----------------------------------------------------------------------------------------------------
void PBC3Dbox::setTraineeSoftening() {
  modelSoftening = "trainee";
  DzetaModel = [this](double zeta) -> double { return (zeta - 1.0) / (this->zetaMax - 1.0); };
  zetaDModel = [this](double D) -> double { return D * (this->zetaMax - 1.0) + 1.0; };
}

/// ----------------------------------------------------------------------------------------------------
/// ----------------------------------------------------------------------------------------------------
void PBC3Dbox::setLinearSoftening() {
  modelSoftening = "linear";
  DzetaModel = [this](double zeta) -> double {
    return (this->zetaMax * (zeta - 1.0)) / ((this->zetaMax - 1.0) * zeta);
  };
  zetaDModel = [this](double D) -> double { return this->zetaMax / (this->zetaMax - D * (this->zetaMax - 1.0)); };
}

/// ----------------------------------------------------------------------------------------------------
/// ----------------------------------------------------------------------------------------------------
void PBC3Dbox::setGateSoftening() {
  modelSoftening = "gate";
  DzetaModel = [this](double zeta) -> double {
    if (zeta <= this->zetaInter) {
      return (zeta - 1.0) / zeta;
    }
    return 1.0 - (this->zetaMax - zeta) / ((this->zetaMax - this->zetaInter) * zeta);
  };
  zetaDModel = [this](double D) -> double {
    double Dinter = (this->zetaInter - 1.0) / this->zetaInter;
    if (D <= Dinter) {
      return 1.0 / (1.0 - D);
    }
    return this->zetaMax / (1.0 + (1.0 - D) * (this->zetaMax - this->zetaInter));
  };
}

/// ----------------------------------------------------------------------------------------------------
/// ----------------------------------------------------------------------------------------------------
double PBC3Dbox::YieldFuncDam(double zeta, double Dn, double DtNorm, double DrotNorm) {
  START_TIMER("YieldFuncDam");
  double yieldFunc;
  if (drot0 > 0.0) {
    yieldFunc = Dn / (zeta * dn0) + pow(DtNorm / (zeta * dt0), powSurf) + pow(DrotNorm / (zeta * drot0), powSurf) - 1.0;
  } else {
    yieldFunc = Dn / (zeta * dn0) + pow(DtNorm / (zeta * dt0), powSurf) - 1.0;
  }
  return yieldFunc;
}

/// ----------------------------------------------------------------------------------------------------
/// ----------------------------------------------------------------------------------------------------
void PBC3Dbox::addKineticStress() {
  for (size_t i = 0; i < Particles.size(); i++) {
    // Remark: only the fluctuating part of velocities is part of the
    //         kinetic stress
    vec3r vel = Cell.h * Particles[i].vel;  // + Cell.vh * Particles[i].pos;
    Sig.xx += Particles[i].mass * vel.x * vel.x;
    Sig.xy += Particles[i].mass * vel.x * vel.y;
    Sig.xz += Particles[i].mass * vel.x * vel.z;
    Sig.yx += Particles[i].mass * vel.y * vel.x;
    Sig.yy += Particles[i].mass * vel.y * vel.y;
    Sig.yz += Particles[i].mass * vel.y * vel.z;
    Sig.zx += Particles[i].mass * vel.z * vel.x;
    Sig.zy += Particles[i].mass * vel.z * vel.y;
    Sig.zz += Particles[i].mass * vel.z * vel.z;
  }
}

/// ----------------------------------------------------------------------------------------------------
/// @brief Computes the interaction forces and moments,
///        and the tensorial moment (= Vcell * stress matrix) of the cell
/// ----------------------------------------------------------------------------------------------------
void PBC3Dbox::computeForcesAndMoments() {
  START_TIMER("computeForcesAndMoments");

  size_t i, j;
  for (size_t k = 0; k < Interactions.size(); k++) {
    i = Interactions[k].i;
    j = Interactions[k].j;

    vec3r sij = Particles[j].pos - Particles[i].pos;
    vec3r imag_j_period_move(floor(sij.x + 0.5), floor(sij.y + 0.5), floor(sij.z + 0.5));
    sij -= imag_j_period_move;
    vec3r branch = Cell.h * sij;

    if (Interactions[k].state == bondedState) {
      // ===========================================================
      // ============== A BONDED ELASTIC INTERACTION ===============
      // ===========================================================

      nbActiveInteractions++;
      nbBonds++;

      // Current normal vector (the previous one has previously been stored in Interactions[k].n)
      vec3r n = branch;
      double len = n.normalize();

      // real relative velocities
      vec3r vel = Particles[j].vel - Particles[i].vel;
      vec3r realVel = Cell.h * vel + Cell.vh * sij;
      realVel -= Particles[i].radius * cross(n, Particles[i].vrot) + Particles[j].radius * cross(n, Particles[j].vrot);

      // Normal force (elastic + viscuous damping)
      double dn = len - (Particles[i].radius + Particles[j].radius) -
                  Interactions[k].gap0;          // normal distance (a negative value is an overlap)
      double vn = realVel * n;                   // normal relative (j/i) velocity
      double fne = -kn * dn;                     // elastic normal force component
      double fnv = -Interactions[k].dampn * vn;  // viscuous normal force component
      Interactions[k].fn = fne + fnv;            // normal force component

      // Tangential force (elastic without viscuous damping)
      vec3r ft_corr = Interactions[k].ft;  // force that will be corrected
      if (objectiveFriction == 1) {
        ft_corr -= cross(ft_corr, cross(Interactions[k].n, n));                               // 1st correction
        ft_corr -= cross(ft_corr, (dt_2 * (Particles[i].vrot + Particles[j].vrot) * n) * n);  // 2nd correction
      }
      vec3r vt = realVel - (vn * n);                // relative velocity projected on the tangential plan
      Interactions[k].ft = ft_corr - kt * vt * dt;  // increment the tangential force

      // Torque (elastic withour viscuous damping)
      Interactions[k].mom -= kr * (Particles[j].vrot - Particles[i].vrot) * dt;  // elastic moment

      // Rupture criterion
      double yieldFunc;
      if (mom0 > 0) {
        yieldFunc = pow(norm(Interactions[k].ft) / ft0, powSurf) + pow(norm(Interactions[k].mom) / mom0, powSurf) -
                    Interactions[k].fn / fn0 - 1.0;
      } else {
        yieldFunc = pow(norm(Interactions[k].ft) / ft0, powSurf) - Interactions[k].fn / fn0 - 1.0;
      }
      if (yieldFunc > 0.0) {
        if (Interactions[k].fn < 0.0) {
          Interactions[k].state = noContactState;
          Interactions[k].fn = 0.0;
          Interactions[k].ft.reset();
          Interactions[k].mom.reset();
          tensfailure++;
        } else {
          Interactions[k].state = contactState;
          Interactions[k].ft.reset();  // PROVISOIR (TODO-> activer frottement)
          Interactions[k].mom.reset();
          fricfailure++;
        }
      }

      // Resultant forces
      vec3r f = Interactions[k].fn * n + Interactions[k].ft;
      Particles[i].force -= f;
      Particles[j].force += f;

      // Resultant moments
      vec3r Ci = (Particles[i].radius + 0.5 * dn) * n;
      vec3r Cj = -(Particles[j].radius + 0.5 * dn) * n;
      Particles[i].moment += cross(Ci, f) - Interactions[k].mom;
      Particles[j].moment += cross(Cj, -f) + Interactions[k].mom;

      // Internal stress
      Sig.xx += f.x * branch.x;
      Sig.xy += f.x * branch.y;
      Sig.xz += f.x * branch.z;

      Sig.yx += f.y * branch.x;
      Sig.yy += f.y * branch.y;
      Sig.yz += f.y * branch.z;

      Sig.zx += f.z * branch.x;
      Sig.zy += f.z * branch.y;
      Sig.zz += f.z * branch.z;

      // Store the normal vector
      Interactions[k].n = n;

    }  // end if bonded interaction
    else if (Interactions[k].state == bondedStateDam) {
      // ===========================================================
      // ========= A BONDED ELASTIC-DAMAGEABLE INTERACTION =========
      // =========    WITH SEPARATED CONTACT AND BOND      =========
      // ===========================================================

      nbActiveInteractions++;
      nbBonds++;

      // Current normal vector (the previous one has previously been stored in Interactions[k].n)
      vec3r n = branch;
      double len = n.normalize();

      // real relative velocities
      vec3r vel = Particles[j].vel - Particles[i].vel;
      vec3r realVel = Cell.h * vel + Cell.vh * sij;  //+ Cell.vh * imag_j_period_move;
      realVel -= Particles[i].radius * cross(n, Particles[i].vrot) + Particles[j].radius * cross(n, Particles[j].vrot);

      // === Normal force (elastic-contact + elastic-damageable-bond + viscuous damping)
      double dn = len - (Particles[i].radius + Particles[j].radius);  // a negative value is an overlap
      double dn_bond = dn - Interactions[k].gap0;  // for a bond, the gap-at-creation is the normal distance reference
      if (continuumContact == 1) {
        dn -= Interactions[k].gap0;
      }  // this gap0 is accounted for only for fake grains
      double vn = realVel * n;  // normal relative (j/i) velocity
      Interactions[k].fn_elas = 0.0;
      if (dn < 0.0) {
        Interactions[k].fn_elas = -w_particle * kn * dn;
      }  // elastic normal contact-force
      Interactions[k].fn_bond = -w_bond * (1.0 - Interactions[k].D) * kn * dn_bond;  // elastic normal bond-force
      double fnv = -Interactions[k].dampn * vn;  // viscuous normal force (not stored)
      Interactions[k].fn = Interactions[k].fn_elas + Interactions[k].fn_bond + fnv;
      double fnUsedForThresholds = Mth::keepPositive(Interactions[k].fn);  // this fn is cut to zero

      // === Tangential force
      vec3r vt = realVel - (vn * n);        // relative velocity projected on the tangential plan
      vec3r deltat = vt * dt;               // increment of tangent relative displacement in the current time-step
      Interactions[k].dt_bond += deltat;    // update the total tangent relative displacement for the bond
      if (dn < 0.0) {                       // if the grains touch each other
        Interactions[k].dt_fric += deltat;  // update the total tangent relative displacement for the friction
        Interactions[k].ft_fric -= w_particle * kt * deltat;  // update the friction force with elastic increment
        double threshold = mu * fnUsedForThresholds;          // could be mu * Interactions[k].fn_elas
        double ft_square = Interactions[k].ft_fric * Interactions[k].ft_fric;
        if (ft_square > 0.0 && ft_square >= threshold * threshold) {
          Interactions[k].ft_fric = threshold * Interactions[k].ft_fric * (1.0 / sqrt(ft_square));
        }
      } else {
        Interactions[k].ft_fric.reset();
        Interactions[k].dt_fric.reset();
      }
      Interactions[k].ft_bond = -w_bond * (1.0 - Interactions[k].D) * kt * Interactions[k].dt_bond;
      Interactions[k].ft = Interactions[k].ft_fric + Interactions[k].ft_bond;

      // === Torque (elastic-plastic without viscuous damping)
      vec3r drot = (Particles[j].vrot - Particles[i].vrot) * dt;
      Interactions[k].drot_bond += drot;
      if (dn < 0.0) {
        Interactions[k].drot_fric += drot;
        Interactions[k].mom_fric -= w_particle * kr * drot;
        double thresholdr = mur * fnUsedForThresholds;
        double mom_square = Interactions[k].mom_fric * Interactions[k].mom_fric;
        if (mom_square > 0.0 && mom_square >= thresholdr * thresholdr) {
          Interactions[k].mom_fric = thresholdr * Interactions[k].mom_fric * (1.0 / sqrt(mom_square));
        }
      } else {
        Interactions[k].drot_fric.reset();
        Interactions[k].mom_fric.reset();
      }
      Interactions[k].mom_bond = -w_bond * (1.0 - Interactions[k].D) * kr * Interactions[k].drot_bond;

      Interactions[k].mom = Interactions[k].mom_fric + Interactions[k].mom_bond;

      // === Update of the damage parameter D + Rupture criterion
      double norm_dt_bond = norm(Interactions[k].dt_bond);
      double norm_drot_bond = norm(Interactions[k].drot_bond);
      double currentZeta = zetaDModel(Interactions[k].D);
      double yieldFunc0 = YieldFuncDam(currentZeta, dn_bond, norm_dt_bond, norm_drot_bond);
      double yieldFuncMax = YieldFuncDam(zetaMax, dn_bond, norm_dt_bond, norm_drot_bond);

      /// UPDATE THE DAMAGE PARAMETER
      if (yieldFuncMax > 0.0) {

        Interactions[k].D = 1.0;

      } else if (yieldFunc0 > 0) {

        double zeta1 = currentZeta;  // set the previous zeta as the first bound
        double zeta2 = zetaMax;      // set the max zeta as the second bound
        double tol = 1.0e-3;
        double surfFunc = yieldFunc0;
        double zetaTest;  // the trial variable

        // dichotomy to find zetaTest that corresponds to |yieldFunc| < tol
        // maximum number of iterations is 20
        for (int p = 0; p < 20; p++) {
          zetaTest = 0.5 * (zeta1 + zeta2);
          surfFunc = YieldFuncDam(zetaTest, dn_bond, norm_dt_bond, norm_drot_bond);

          if (surfFunc < 0.0) {
            zeta2 = zetaTest;
          } else if (surfFunc > 0.0) {
            zeta1 = zetaTest;
          }

          if (fabs(surfFunc) < tol) {
            break;
          }
        }

        if (zetaMax > 1.0) {
          Interactions[k].D = DzetaModel(zetaTest);
        } else {
          Interactions[k].D = 1.0;
        }
      }

      // RUPTURE
      if (Interactions[k].D >= 1.0) {
        if (Interactions[k].fn_elas == 0.0) {  // in case of absence of contact
          Interactions[k].state = noContactState;
          Interactions[k].fn_bond = 0.0;
          Interactions[k].ft_bond.reset();
          Interactions[k].mom_bond.reset();

          Interactions[k].ft_fric.reset();
          Interactions[k].mom_fric.reset();

          Interactions[k].fn = 0.0;
          Interactions[k].ft.reset();
          Interactions[k].mom.reset();

          tensfailure++;
        } else if (Interactions[k].fn_elas > 0.0) {  // in case the particles were in contact
          Interactions[k].state = contactState;
          Interactions[k].fn_bond = 0.0;
          Interactions[k].ft_bond.reset();
          Interactions[k].mom_bond.reset();

          Interactions[k].fn = Interactions[k].fn_elas;
          Interactions[k].ft = Interactions[k].ft_fric;
          Interactions[k].mom = Interactions[k].mom_fric;

          fricfailure++;
        }
      }

      // === Resultant forces
      vec3r f = Interactions[k].fn * n + Interactions[k].ft;
      Particles[i].force -= f;
      Particles[j].force += f;

      // === Resultant moments
      vec3r Ci = (Particles[i].radius + 0.5 * dn) * n;
      vec3r Cj = -(Particles[j].radius + 0.5 * dn) * n;
      Particles[i].moment += cross(Ci, f) - Interactions[k].mom;
      Particles[j].moment += cross(Cj, -f) + Interactions[k].mom;

      // === Internal stress
      Sig.xx += f.x * branch.x;
      Sig.xy += f.x * branch.y;
      Sig.xz += f.x * branch.z;

      Sig.yx += f.y * branch.x;
      Sig.yy += f.y * branch.y;
      Sig.yz += f.y * branch.z;

      Sig.zx += f.z * branch.x;
      Sig.zy += f.z * branch.y;
      Sig.zz += f.z * branch.z;

      // Store the normal vector
      Interactions[k].n = n;
    } else {
      // ===========================================================
      // ======= A NON-BONDED FRICTIONAL CONTACT INTERACTION =======
      // ===========================================================

      double sum = Particles[i].radius + Particles[j].radius;
      if (norm2(branch) <= sum * sum) {  // it means that particles i and j are in contact
        nbActiveInteractions++;
        Interactions[k].state = contactState;

        // Current normal vector (the previous one has previously been stored in Interactions[k].n)
        vec3r n = branch;
        double len = n.normalize();

        // real relative velocities
        vec3r vel = Particles[j].vel - Particles[i].vel;
        vec3r realVel = Cell.h * vel + Cell.vh * sij;
        realVel -=
            Particles[i].radius * cross(n, Particles[i].vrot) + Particles[j].radius * cross(n, Particles[j].vrot);

        // Normal force (elastic + viscuous damping)
        double dn = len - Particles[i].radius - Particles[j].radius;
        double vn = realVel * n;
        double fne = -w_particle * kn * dn;
        double fnv = -Interactions[k].dampn * vn;
        Interactions[k].fn = fne + fnv;
        if (Interactions[k].fn < 0.0) {
          Interactions[k].fn = 0.0;  // Because viscuous damping can make fn negative
        }
        Interactions[k].fn_elas = Interactions[k].fn;

        vec3r ft_corr = Interactions[k].ft;  // force that will be corrected
        if (objectiveFriction == 1) {
          ft_corr -= cross(ft_corr, cross(Interactions[k].n, n));                               // 1st correction
          ft_corr -= cross(ft_corr, (dt_2 * (Particles[i].vrot + Particles[j].vrot) * n) * n);  // 2nd correction
        }
        vec3r vt = realVel - (vn * n);  // relative velocity projected on the tangential plan
        vec3r deltat = vt * dt;
        Interactions[k].dt_fric += deltat;
        Interactions[k].ft = ft_corr - w_particle * kt * deltat;  // no viscuous damping since friction can dissipate
        double threshold = fabs(mu * Interactions[k].fn);         // Suppose that fn is elastic and without cohesion
        double ft_square = Interactions[k].ft * Interactions[k].ft;
        if (ft_square > 0.0 && ft_square >= threshold * threshold) {
          Interactions[k].ft = threshold * Interactions[k].ft * (1.0f / sqrt(ft_square));
        }
        Interactions[k].ft_fric = Interactions[k].ft;

        // moment of resistance
        vec3r drot = (Particles[j].vrot - Particles[i].vrot) * dt;
        Interactions[k].drot_fric += drot;
        Interactions[k].mom_fric -= kr * drot;
        double thresholdr = fabs(mur * Interactions[k].fn);
        double mom_square = Interactions[k].mom_fric * Interactions[k].mom_fric;
        if (mom_square > 0.0 && mom_square >= thresholdr * thresholdr) {
          Interactions[k].mom_fric = thresholdr * Interactions[k].mom_fric * (1.0f / sqrt(mom_square));
        }
        Interactions[k].mom = Interactions[k].mom_fric;

        // Cohesion
        Interactions[k].fn += fcoh;

        // Resultant forces
        vec3r f = Interactions[k].fn * n + Interactions[k].ft;
        Particles[i].force -= f;
        Particles[j].force += f;

        // Resultant moments
        vec3r Ci = (Particles[i].radius + 0.5 * dn) * n;
        vec3r Cj = -(Particles[j].radius + 0.5 * dn) * n;
        Particles[i].moment += cross(Ci, f);
        Particles[j].moment += cross(Cj, -f);

        // Internal stress
        Sig.xx += f.x * branch.x;
        Sig.xy += f.x * branch.y;
        Sig.xz += f.x * branch.z;

        Sig.yx += f.y * branch.x;
        Sig.yy += f.y * branch.y;
        Sig.yz += f.y * branch.z;

        Sig.zx += f.z * branch.x;
        Sig.zy += f.z * branch.y;
        Sig.zz += f.z * branch.z;

        // Store the normal vector
        Interactions[k].n = n;

        if (permamentGluer == 1) {
          // switch to a cemented/bonded link
          Interactions[k].state = bondedStateDam;

          if (dn >= 0.0) {
            Interactions[k].gap0 = dn;
          } else {
            Interactions[k].gap0 = 0.0;
          }
        }
      } else {
        Interactions[k].dt_fric.reset();
        Interactions[k].mom_fric.reset();
        Interactions[k].mom.reset();
        Interactions[k].state = noContactState;
      }
    }  // ====== end if non-bonded interactions

  }  // Loop over interactions
}

// =======================================================================
//             METHODS FOR MPMxDEM COUPLING
// =======================================================================

void PBC3Dbox::transform(mat9r& Finc, double macro_dt, int nstepMin, double rateAverage, double rateCriticalTimeStep,
                         mat9r& SigAvg) {
  computeSampleData();  // this will compute the smallest particle volume Vmin
  double dtc = M_PI * sqrt(Vmin * density / kn);
  double beginavg = macro_dt * (1.0 - rateAverage);
  dt = dtc * rateCriticalTimeStep;

  // Restrict to a minimum of nstepMin DEM-time-increments
  double dtMax = macro_dt / (double)nstepMin;
  if (dt >= dtMax) {
    dt = dtMax;
  }

  // Now, we've got the good time-step
  dt_2 = 0.5 * dt;
  dt2_2 = 0.5 * dt * dt;
  t = 0.0;
  tmax = macro_dt;

  // Define the transformation velocity matrix vh
  mat9r dFmI = Finc;
  dFmI.xx -= 1.0;
  dFmI.yy -= 1.0;
  dFmI.zz -= 1.0;
  Cell.vh = (1.0f / macro_dt) * (dFmI * Cell.h);
  Load.VelocityControl(Cell.vh);

  // Set the time-period for rebuilding the verlet list
  // interVerlet = 5.0 * dt;  // on peut faire une meilleur estimation
  // clang-format off
  double vmax = std::max({fabs(Cell.vh.xx), fabs(Cell.vh.xy), fabs(Cell.vh.xz), 
		                      fabs(Cell.vh.yx), fabs(Cell.vh.yy), fabs(Cell.vh.yz), 
													fabs(Cell.vh.zx), fabs(Cell.vh.zy), fabs(Cell.vh.zz)});
  // clang-format on

  // vmax * interVerlet = 0.5 * Rmin
  interVerlet = 0.5 * Rmin / vmax;
  if (interVerlet > macro_dt)
    interVerlet = macro_dt;
  else if (interVerlet < dt)
    interVerlet = dt;

  updateNeighborList(dVerlet);
  interVerletC = 0;

  accelerations();

  std::vector<double> sxx;
  std::vector<double> sxy;
  std::vector<double> sxz;
  std::vector<double> syx;
  std::vector<double> syy;
  std::vector<double> syz;
  std::vector<double> szx;
  std::vector<double> szy;
  std::vector<double> szz;
  std::vector<double> tvec;

  while (t < tmax) {
    computeSampleData();
    velocityVerletStep();

    if (t >= beginavg - dt) {
      tvec.push_back(t);
      sxx.push_back(Sig.xx);
      sxy.push_back(Sig.xy);
      sxz.push_back(Sig.xz);
      syx.push_back(Sig.yx);
      syy.push_back(Sig.yy);
      syz.push_back(Sig.yz);
      szx.push_back(Sig.zx);
      szy.push_back(Sig.zy);
      szz.push_back(Sig.zz);
    }

    if (interVerletC >= interVerlet) {
      updateNeighborList(dVerlet);
      interVerletC = 0.0;
    }

    interVerletC += dt;
    t += dt;
  }

  linreg* Regr = linreg::get();
  double tlast = t - dt;
  if (sxx.size() > 1) {
    Regr->run(tvec, sxx);
    SigAvg.xx = Regr->orig + tlast * Regr->slope;
    Regr->run(tvec, sxy);
    SigAvg.xy = Regr->orig + tlast * Regr->slope;
    Regr->run(tvec, sxz);
    SigAvg.xz = Regr->orig + tlast * Regr->slope;
    Regr->run(tvec, syx);
    SigAvg.yx = Regr->orig + tlast * Regr->slope;
    Regr->run(tvec, syy);
    SigAvg.yy = Regr->orig + tlast * Regr->slope;
    Regr->run(tvec, syz);
    SigAvg.yz = Regr->orig + tlast * Regr->slope;
    Regr->run(tvec, szx);
    SigAvg.zx = Regr->orig + tlast * Regr->slope;
    Regr->run(tvec, szy);
    SigAvg.zy = Regr->orig + tlast * Regr->slope;
    Regr->run(tvec, szz);
    SigAvg.zz = Regr->orig + tlast * Regr->slope;
  } else {
    SigAvg = Sig;
  }
}

// P transformation is supposed to be a rotation where the result frame is still direct
// Pas si facile à écrire ce genre de fonction...
void PBC3Dbox::applySwitchMatrix(mat9r& P) {
  // Switch the particles
  for (size_t i = 0; i < Particles.size(); i++) {
    Particles[i].pos = P * Particles[i].pos;
    for (size_t c = 0; c < 3; c++) {
      while (Particles[i].pos[c] < 0.0) {
        Particles[i].pos[c] += 1.0;
      }
      while (Particles[i].pos[c] > 1.0) {
        Particles[i].pos[c] -= 1.0;
      }
    }

    Particles[i].vel = P * Particles[i].vel;
    Particles[i].acc = P * Particles[i].acc;

    // Q ???
    Particles[i].vrot = P * Particles[i].vrot;
    Particles[i].arot = P * Particles[i].arot;
  }

  // Switch the interactions
  // TODO

  // Switch the periodic cell
  // TODO
}

// dirNum = 0, 1 or 2 for resp. the first, second and third column of the h matrix
void PBC3Dbox::ModularTransformation() {
  /*
  vec3r translate;
  switch (dirNum) {
          case 0: {
                  translate = h.get_xcol();
          } break;
          case 1: {
                  translate = h.get_ycol();
          } break;
          case 2: {
                  translate = h.get_zcol();
          } break;
          default:{
                  std::cerr << "@ModularTransformation, dirNum = 0, 1 or 2\n";
          }
  }
  */

  mat9r new_h;
  mat9r new_vh;
}

// =======================================================================
//             METHODS FOR COUPLING WITH LAGAMINE (FEMxDEM)
// =======================================================================

void PBC3Dbox::getOperatorKruyt(double L[6][6]) {
  for (size_t i = 0; i < 6; i++) {
    for (size_t j = 0; j < 6; j++) {
      L[i][j] = 0.0;
    }
  }

  // precomputations
  double Vcell = fabs(Cell.h.det());
  double kn_Vcell = kn / Vcell;
  double kt_Vcell = kt / Vcell;

  // indexes
  const size_t xx = 0;
  const size_t yy = 1;
  const size_t zz = 2;
  const size_t xy = 3;
  const size_t xz = 4;
  const size_t yz = 5;

  for (size_t k = 0; k < Interactions.size(); k++) {
    if (Interactions[k].state == noContactState) {
      continue;
    }

    size_t i = Interactions[k].i;
    size_t j = Interactions[k].j;

    vec3r sij = Particles[j].pos - Particles[i].pos;
    sij.x -= floor(sij.x + 0.5);
    sij.y -= floor(sij.y + 0.5);
    sij.z -= floor(sij.z + 0.5);
    vec3r l = Cell.h * sij;
    vec3r n = l;
    n.normalize();
    vec3r T = Interactions[k].ft;
    T.normalize();

    L[xx][xx] += kn_Vcell * n.x * l.x * n.x * l.x + kt_Vcell * T.x * l.x * T.x * l.x;
    L[xx][yy] += kn_Vcell * n.x * l.x * n.y * l.y + kt_Vcell * T.x * l.x * T.y * l.y;
    L[xx][zz] += kn_Vcell * n.x * l.x * n.z * l.z + kt_Vcell * T.x * l.x * T.z * l.z;
    L[xx][xy] += kn_Vcell * n.x * l.x * n.x * l.y + kt_Vcell * T.x * l.x * T.x * l.y;
    L[xx][xz] += kn_Vcell * n.x * l.x * n.x * l.z + kt_Vcell * T.x * l.x * T.x * l.z;
    L[xx][yz] += kn_Vcell * n.x * l.x * n.y * l.z + kt_Vcell * T.x * l.x * T.y * l.z;

    L[yy][xx] += kn_Vcell * n.y * l.y * n.x * l.x + kt_Vcell * T.y * l.y * T.x * l.x;
    L[yy][yy] += kn_Vcell * n.y * l.y * n.y * l.y + kt_Vcell * T.y * l.y * T.y * l.y;
    L[yy][zz] += kn_Vcell * n.y * l.y * n.z * l.z + kt_Vcell * T.y * l.y * T.z * l.z;
    L[yy][xy] += kn_Vcell * n.y * l.y * n.x * l.y + kt_Vcell * T.y * l.y * T.x * l.y;
    L[yy][xz] += kn_Vcell * n.y * l.y * n.x * l.z + kt_Vcell * T.y * l.y * T.x * l.z;
    L[yy][yz] += kn_Vcell * n.y * l.y * n.y * l.z + kt_Vcell * T.y * l.y * T.y * l.z;

    L[zz][xx] += kn_Vcell * n.z * l.z * n.x * l.x + kt_Vcell * T.z * l.z * T.x * l.x;
    L[zz][yy] += kn_Vcell * n.z * l.z * n.y * l.y + kt_Vcell * T.z * l.z * T.y * l.y;
    L[zz][zz] += kn_Vcell * n.z * l.z * n.z * l.z + kt_Vcell * T.z * l.z * T.z * l.z;
    L[zz][xy] += kn_Vcell * n.z * l.z * n.x * l.y + kt_Vcell * T.z * l.z * T.x * l.y;
    L[zz][xz] += kn_Vcell * n.z * l.z * n.x * l.z + kt_Vcell * T.z * l.z * T.x * l.z;
    L[zz][yz] += kn_Vcell * n.z * l.z * n.y * l.z + kt_Vcell * T.z * l.z * T.y * l.z;

    L[xy][xx] += kn_Vcell * n.x * l.y * n.x * l.x + kt_Vcell * T.x * l.y * T.x * l.x;
    L[xy][yy] += kn_Vcell * n.x * l.y * n.y * l.y + kt_Vcell * T.x * l.y * T.y * l.y;
    L[xy][zz] += kn_Vcell * n.x * l.y * n.z * l.z + kt_Vcell * T.x * l.y * T.z * l.z;
    L[xy][xy] += kn_Vcell * n.x * l.y * n.x * l.y + kt_Vcell * T.x * l.y * T.x * l.y;
    L[xy][xz] += kn_Vcell * n.x * l.y * n.x * l.z + kt_Vcell * T.x * l.y * T.x * l.z;
    L[xy][yz] += kn_Vcell * n.x * l.y * n.y * l.z + kt_Vcell * T.x * l.y * T.y * l.z;

    L[xz][xx] += kn_Vcell * n.x * l.z * n.x * l.x + kt_Vcell * T.x * l.z * T.x * l.x;
    L[xz][yy] += kn_Vcell * n.x * l.z * n.y * l.y + kt_Vcell * T.x * l.z * T.y * l.y;
    L[xz][zz] += kn_Vcell * n.x * l.z * n.z * l.z + kt_Vcell * T.x * l.z * T.z * l.z;
    L[xz][xy] += kn_Vcell * n.x * l.z * n.x * l.y + kt_Vcell * T.x * l.z * T.x * l.y;
    L[xz][xz] += kn_Vcell * n.x * l.z * n.x * l.z + kt_Vcell * T.x * l.z * T.x * l.z;
    L[xz][yz] += kn_Vcell * n.x * l.z * n.y * l.z + kt_Vcell * T.x * l.z * T.y * l.z;

    L[yz][xx] += kn_Vcell * n.y * l.z * n.x * l.x + kt_Vcell * T.y * l.z * T.x * l.x;
    L[yz][yy] += kn_Vcell * n.y * l.z * n.y * l.y + kt_Vcell * T.y * l.z * T.y * l.y;
    L[yz][zz] += kn_Vcell * n.y * l.z * n.z * l.z + kt_Vcell * T.y * l.z * T.z * l.z;
    L[yz][xy] += kn_Vcell * n.y * l.z * n.x * l.y + kt_Vcell * T.y * l.z * T.x * l.y;
    L[yz][xz] += kn_Vcell * n.y * l.z * n.x * l.z + kt_Vcell * T.y * l.z * T.x * l.z;
    L[yz][yz] += kn_Vcell * n.y * l.z * n.y * l.z + kt_Vcell * T.y * l.z * T.y * l.z;
  }
}

// This 2nd version is a little bit more "generic".
// In the first version the components xy, xz and yz are missing
void PBC3Dbox::getOperatorKruyt2(double L[9][9]) {
  for (size_t i = 0; i < 9; i++) {
    for (size_t j = 0; j < 9; j++) {
      L[i][j] = 0.0;
    }
  }

  // precomputations
  double Vcell = fabs(Cell.h.det());
  double kn_Vcell = 0.93 * kn / Vcell;
  double kt_Vcell = 0.63 * kt / Vcell;

  // indexes
  const size_t xx = 0;
  const size_t yy = 1;
  const size_t zz = 2;
  const size_t xy = 3;
  const size_t xz = 4;
  const size_t yz = 5;
  const size_t yx = 6;
  const size_t zx = 7;
  const size_t zy = 8;

  double nx, ny, nz, tx, ty, tz, wx, wy, wz;

  for (size_t k = 0; k < Interactions.size(); k++) {
    if (Interactions[k].state == noContactState) {
      continue;
    }

    size_t i = Interactions[k].i;
    size_t j = Interactions[k].j;

    vec3r sij = Particles[j].pos - Particles[i].pos;
    sij.x -= floor(sij.x + 0.5);
    sij.y -= floor(sij.y + 0.5);
    sij.z -= floor(sij.z + 0.5);
    vec3r l = Cell.h * sij;
    vec3r n = l;
    n.normalize();
    // vec3r t = Interactions[k].ft;
    // t.normalize();

    // Noraml vector n
    nx = n.x;
    ny = n.y;
    nz = n.z;

    // vector t
    tx = ny / sqrt(nx * nx + ny * ny);
    ty = -nx / sqrt(nx * nx + ny * ny);
    tz = 0.0;

    // vector w
    wx = ny * tz - nz * ty;
    wy = nz * tx - nx * tz;
    wz = nx * ty - ny * tx;

    // xx
    L[xx][xx] += kn_Vcell * nx * l.x * nx * l.x + kt_Vcell * tx * l.x * tx * l.x + kt_Vcell * wx * l.x * wx * l.x;  // 1
    L[xx][yy] += kn_Vcell * nx * l.x * ny * l.y + kt_Vcell * tx * l.x * ty * l.y + kt_Vcell * wx * l.x * wy * l.y;  // 2
    L[xx][zz] += kn_Vcell * nx * l.x * nz * l.z + kt_Vcell * tx * l.x * tz * l.z + kt_Vcell * wx * l.x * wz * l.z;  // 3
    L[xx][xy] += kn_Vcell * nx * l.x * nx * l.y + kt_Vcell * tx * l.x * tx * l.y + kt_Vcell * wx * l.x * wx * l.y;  // 4
    L[xx][xz] += kn_Vcell * nx * l.x * nx * l.z + kt_Vcell * tx * l.x * tx * l.z + kt_Vcell * wx * l.x * wx * l.z;  // 5
    L[xx][yz] += kn_Vcell * nx * l.x * ny * l.z + kt_Vcell * tx * l.x * ty * l.z + kt_Vcell * wx * l.x * wy * l.z;  // 6
    L[xx][yx] += kn_Vcell * nx * l.x * ny * l.x + kt_Vcell * tx * l.x * ty * l.x + kt_Vcell * wx * l.x * wy * l.x;  // 7
    L[xx][zx] += kn_Vcell * nx * l.x * nz * l.x + kt_Vcell * tx * l.x * tz * l.x + kt_Vcell * wx * l.x * wz * l.x;  // 8
    L[xx][zy] += kn_Vcell * nx * l.x * nz * l.y + kt_Vcell * tx * l.x * tz * l.y + kt_Vcell * wx * l.x * wz * l.y;  // 9

    // yy
    L[yy][xx] += kn_Vcell * ny * l.y * nx * l.x + kt_Vcell * ty * l.y * tx * l.x + kt_Vcell * wy * l.y * wx * l.x;  // 1
    L[yy][yy] += kn_Vcell * ny * l.y * ny * l.y + kt_Vcell * ty * l.y * ty * l.y + kt_Vcell * wy * l.y * wy * l.y;  // 2
    L[yy][zz] += kn_Vcell * ny * l.y * nz * l.z + kt_Vcell * ty * l.y * tz * l.z + kt_Vcell * wy * l.y * wz * l.z;  // 3
    L[yy][xy] += kn_Vcell * ny * l.y * nx * l.y + kt_Vcell * ty * l.y * tx * l.y + kt_Vcell * wy * l.y * wx * l.y;  // 4
    L[yy][xz] += kn_Vcell * ny * l.y * nx * l.z + kt_Vcell * ty * l.y * tx * l.z + kt_Vcell * wy * l.y * wx * l.z;  // 5
    L[yy][yz] += kn_Vcell * ny * l.y * ny * l.z + kt_Vcell * ty * l.y * ty * l.z + kt_Vcell * wy * l.y * wy * l.z;  // 6
    L[yy][yx] += kn_Vcell * ny * l.y * ny * l.x + kt_Vcell * ty * l.y * ty * l.x + kt_Vcell * wy * l.y * wy * l.x;  // 7
    L[yy][zx] += kn_Vcell * ny * l.y * nz * l.x + kt_Vcell * ty * l.y * tz * l.x + kt_Vcell * wy * l.y * wz * l.x;  // 8
    L[yy][zy] += kn_Vcell * ny * l.y * nz * l.y + kt_Vcell * ty * l.y * tz * l.y + kt_Vcell * wy * l.y * wz * l.y;  // 9

    // zz
    L[zz][xx] += kn_Vcell * nz * l.z * nx * l.x + kt_Vcell * tz * l.z * tx * l.x + kt_Vcell * wz * l.z * wx * l.x;  // 1
    L[zz][yy] += kn_Vcell * nz * l.z * ny * l.y + kt_Vcell * tz * l.z * ty * l.y + kt_Vcell * wz * l.z * wy * l.y;  // 2
    L[zz][zz] += kn_Vcell * nz * l.z * nz * l.z + kt_Vcell * tz * l.z * tz * l.z + kt_Vcell * wz * l.z * wz * l.z;  // 3
    L[zz][xy] += kn_Vcell * nz * l.z * nx * l.y + kt_Vcell * tz * l.z * tx * l.y + kt_Vcell * wz * l.z * wx * l.y;  // 4
    L[zz][xz] += kn_Vcell * nz * l.z * nx * l.z + kt_Vcell * tz * l.z * tx * l.z + kt_Vcell * wz * l.z * wx * l.z;  // 5
    L[zz][yz] += kn_Vcell * nz * l.z * ny * l.z + kt_Vcell * tz * l.z * ty * l.z + kt_Vcell * wz * l.z * wy * l.z;  // 6
    L[zz][yx] += kn_Vcell * nz * l.z * ny * l.x + kt_Vcell * tz * l.z * ty * l.x + kt_Vcell * wz * l.z * wy * l.x;  // 7
    L[zz][zx] += kn_Vcell * nz * l.z * nz * l.x + kt_Vcell * tz * l.z * tz * l.x + kt_Vcell * wz * l.z * wz * l.x;  // 8
    L[zz][zy] += kn_Vcell * nz * l.z * nz * l.y + kt_Vcell * tz * l.z * tz * l.y + kt_Vcell * wz * l.z * wz * l.y;  // 9

    // xy
    L[xy][xx] += kn_Vcell * nx * l.y * nx * l.x + kt_Vcell * tx * l.y * tx * l.x + kt_Vcell * wx * l.y * wx * l.x;  // 1
    L[xy][yy] += kn_Vcell * nx * l.y * ny * l.y + kt_Vcell * tx * l.y * ty * l.y + kt_Vcell * wx * l.y * wy * l.y;  // 2
    L[xy][zz] += kn_Vcell * nx * l.y * nz * l.z + kt_Vcell * tx * l.y * tz * l.z + kt_Vcell * wx * l.y * wz * l.z;  // 3
    L[xy][xy] += kn_Vcell * nx * l.y * nx * l.y + kt_Vcell * tx * l.y * tx * l.y + kt_Vcell * wx * l.y * wx * l.y;  // 4
    L[xy][xz] += kn_Vcell * nx * l.y * nx * l.z + kt_Vcell * tx * l.y * tx * l.z + kt_Vcell * wx * l.y * wx * l.z;  // 5
    L[xy][yz] += kn_Vcell * nx * l.y * ny * l.z + kt_Vcell * tx * l.y * ty * l.z + kt_Vcell * wx * l.y * wy * l.z;  // 6
    L[xy][yx] += kn_Vcell * nx * l.y * ny * l.x + kt_Vcell * tx * l.y * ty * l.x + kt_Vcell * wx * l.y * wy * l.x;  // 7
    L[xy][zx] += kn_Vcell * nx * l.y * nz * l.x + kt_Vcell * tx * l.y * tz * l.x + kt_Vcell * wx * l.y * wz * l.x;  // 8
    L[xy][zy] += kn_Vcell * nx * l.y * nz * l.y + kt_Vcell * tx * l.y * tz * l.y + kt_Vcell * wx * l.y * wz * l.y;  // 9

    // xz
    L[xz][xx] += kn_Vcell * nx * l.z * nx * l.x + kt_Vcell * tx * l.z * tx * l.x + kt_Vcell * wx * l.z * wx * l.x;  // 1
    L[xz][yy] += kn_Vcell * nx * l.z * ny * l.y + kt_Vcell * tx * l.z * ty * l.y + kt_Vcell * wx * l.z * wy * l.y;  // 2
    L[xz][zz] += kn_Vcell * nx * l.z * nz * l.z + kt_Vcell * tx * l.z * tz * l.z + kt_Vcell * wx * l.z * wz * l.z;  // 3
    L[xz][xy] += kn_Vcell * nx * l.z * nx * l.y + kt_Vcell * tx * l.z * tx * l.y + kt_Vcell * wx * l.z * wx * l.y;  // 4
    L[xz][xz] += kn_Vcell * nx * l.z * nx * l.z + kt_Vcell * tx * l.z * tx * l.z + kt_Vcell * wx * l.z * wx * l.z;  // 5
    L[xz][yz] += kn_Vcell * nx * l.z * ny * l.z + kt_Vcell * tx * l.z * ty * l.z + kt_Vcell * wx * l.z * wy * l.z;  // 6
    L[xz][yx] += kn_Vcell * nx * l.z * ny * l.x + kt_Vcell * tx * l.z * ty * l.x + kt_Vcell * wx * l.z * wy * l.x;  // 7
    L[xz][zx] += kn_Vcell * nx * l.z * nz * l.x + kt_Vcell * tx * l.z * tz * l.x + kt_Vcell * wx * l.z * wz * l.x;  // 8
    L[xz][zy] += kn_Vcell * nx * l.z * nz * l.y + kt_Vcell * tx * l.z * tz * l.y + kt_Vcell * wx * l.z * wz * l.y;  // 9

    // yz
    L[yz][xx] += kn_Vcell * ny * l.z * nx * l.x + kt_Vcell * ty * l.z * tx * l.x + kt_Vcell * wy * l.z * wx * l.x;  // 1
    L[yz][yy] += kn_Vcell * ny * l.z * ny * l.y + kt_Vcell * ty * l.z * ty * l.y + kt_Vcell * wy * l.z * wy * l.y;  // 2
    L[yz][zz] += kn_Vcell * ny * l.z * nz * l.z + kt_Vcell * ty * l.z * tz * l.z + kt_Vcell * wy * l.z * wz * l.z;  // 3
    L[yz][xy] += kn_Vcell * ny * l.z * nx * l.y + kt_Vcell * ty * l.z * tx * l.y + kt_Vcell * wy * l.z * wx * l.y;  // 4
    L[yz][xz] += kn_Vcell * ny * l.z * nx * l.z + kt_Vcell * ty * l.z * tx * l.z + kt_Vcell * wy * l.z * wx * l.z;  // 5
    L[yz][yz] += kn_Vcell * ny * l.z * ny * l.z + kt_Vcell * ty * l.z * ty * l.z + kt_Vcell * wy * l.z * wy * l.z;  // 6
    L[yz][yx] += kn_Vcell * ny * l.z * ny * l.x + kt_Vcell * ty * l.z * ty * l.x + kt_Vcell * wy * l.z * wy * l.x;  // 7
    L[yz][zx] += kn_Vcell * ny * l.z * nz * l.x + kt_Vcell * ty * l.z * tz * l.x + kt_Vcell * wy * l.z * wz * l.x;  // 8
    L[yz][zy] += kn_Vcell * ny * l.z * nz * l.y + kt_Vcell * ty * l.z * tz * l.y + kt_Vcell * wy * l.z * wz * l.y;  // 9

    // yx
    L[yx][xx] += kn_Vcell * ny * l.x * nx * l.x + kt_Vcell * ty * l.x * tx * l.x + kt_Vcell * wy * l.x * wx * l.x;  // 1
    L[yx][yy] += kn_Vcell * ny * l.x * ny * l.y + kt_Vcell * ty * l.x * ty * l.y + kt_Vcell * wy * l.x * wy * l.y;  // 2
    L[yx][zz] += kn_Vcell * ny * l.x * nz * l.z + kt_Vcell * ty * l.x * tz * l.z + kt_Vcell * wy * l.x * wz * l.z;  // 3
    L[yx][xy] += kn_Vcell * ny * l.x * nx * l.y + kt_Vcell * ty * l.x * tx * l.y + kt_Vcell * wy * l.x * wx * l.y;  // 4
    L[yx][xz] += kn_Vcell * ny * l.x * nx * l.z + kt_Vcell * ty * l.x * tx * l.z + kt_Vcell * wy * l.x * wx * l.z;  // 5
    L[yx][yz] += kn_Vcell * ny * l.x * ny * l.z + kt_Vcell * ty * l.x * ty * l.z + kt_Vcell * wy * l.x * wy * l.z;  // 6
    L[yx][yx] += kn_Vcell * ny * l.x * ny * l.x + kt_Vcell * ty * l.x * ty * l.x + kt_Vcell * wy * l.x * wy * l.x;  // 7
    L[yx][zx] += kn_Vcell * ny * l.x * nz * l.x + kt_Vcell * ty * l.x * tz * l.x + kt_Vcell * wy * l.x * wz * l.x;  // 8
    L[yx][zy] += kn_Vcell * ny * l.x * nz * l.y + kt_Vcell * ty * l.x * tz * l.y + kt_Vcell * wy * l.x * wz * l.y;  // 9

    // zx
    L[zx][xx] += kn_Vcell * nz * l.x * nx * l.x + kt_Vcell * tz * l.x * tx * l.x + kt_Vcell * wz * l.x * wx * l.x;  // 1
    L[zx][yy] += kn_Vcell * nz * l.x * ny * l.y + kt_Vcell * tz * l.x * ty * l.y + kt_Vcell * wz * l.x * wy * l.y;  // 2
    L[zx][zz] += kn_Vcell * nz * l.x * nz * l.z + kt_Vcell * tz * l.x * tz * l.z + kt_Vcell * wz * l.x * wz * l.z;  // 3
    L[zx][xy] += kn_Vcell * nz * l.x * nx * l.y + kt_Vcell * tz * l.x * tx * l.y + kt_Vcell * wz * l.x * wx * l.y;  // 4
    L[zx][xz] += kn_Vcell * nz * l.x * nx * l.z + kt_Vcell * tz * l.x * tx * l.z + kt_Vcell * wz * l.x * wx * l.z;  // 5
    L[zx][yz] += kn_Vcell * nz * l.x * ny * l.z + kt_Vcell * tz * l.x * ty * l.z + kt_Vcell * wz * l.x * wy * l.z;  // 6
    L[zx][yx] += kn_Vcell * nz * l.x * ny * l.x + kt_Vcell * tz * l.x * ty * l.x + kt_Vcell * wz * l.x * wy * l.x;  // 7
    L[zx][zx] += kn_Vcell * nz * l.x * nz * l.x + kt_Vcell * tz * l.x * tz * l.x + kt_Vcell * wz * l.x * wz * l.x;  // 8
    L[zx][zy] += kn_Vcell * nz * l.x * nz * l.y + kt_Vcell * tz * l.x * tz * l.y + kt_Vcell * wz * l.x * wz * l.y;  // 9

    // zy
    L[zy][xx] += kn_Vcell * nz * l.y * nx * l.x + kt_Vcell * tz * l.y * tx * l.x + kt_Vcell * wz * l.y * wx * l.x;  // 1
    L[zy][yy] += kn_Vcell * nz * l.y * ny * l.y + kt_Vcell * tz * l.y * ty * l.y + kt_Vcell * wz * l.y * wy * l.y;  // 2
    L[zy][zz] += kn_Vcell * nz * l.y * nz * l.z + kt_Vcell * tz * l.y * tz * l.z + kt_Vcell * wz * l.y * wz * l.z;  // 3
    L[zy][xy] += kn_Vcell * nz * l.y * nx * l.y + kt_Vcell * tz * l.y * tx * l.y + kt_Vcell * wz * l.y * wx * l.y;  // 4
    L[zy][xz] += kn_Vcell * nz * l.y * nx * l.z + kt_Vcell * tz * l.y * tx * l.z + kt_Vcell * wz * l.y * wx * l.z;  // 5
    L[zy][yz] += kn_Vcell * nz * l.y * ny * l.z + kt_Vcell * tz * l.y * ty * l.z + kt_Vcell * wz * l.y * wy * l.z;  // 6
    L[zy][yx] += kn_Vcell * nz * l.y * ny * l.x + kt_Vcell * tz * l.y * ty * l.x + kt_Vcell * wz * l.y * wy * l.x;  // 7
    L[zy][zx] += kn_Vcell * nz * l.y * nz * l.x + kt_Vcell * tz * l.y * tz * l.x + kt_Vcell * wz * l.y * wz * l.x;  // 8
    L[zy][zy] += kn_Vcell * nz * l.y * nz * l.y + kt_Vcell * tz * l.y * tz * l.y + kt_Vcell * wz * l.y * wz * l.y;  // 9
  }
}

// this version 2b is essentially the same as 2 but written in true 4th order rank tensor (easier to apply rotation)
void PBC3Dbox::getOperatorKruyt2b(double L[3][3][3][3]) {
  for (size_t i = 0; i < 3; i++) {
    for (size_t j = 0; j < 3; j++) {
      for (size_t k = 0; k < 3; k++) {
        for (size_t l = 0; l < 3; l++) {
          L[i][j][k][l] = 0.0;
        }
      }
    }
  }

  // precomputations
  double Vcell = fabs(Cell.h.det());
  double kn_Vcell = 0.93 * kn / Vcell;
  double kt_Vcell = 0.63 * kt / Vcell;

  // indexes
  const size_t x = 0;
  const size_t y = 1;
  const size_t z = 2;

  double nx, ny, nz, tx, ty, tz, wx, wy, wz;

  for (size_t k = 0; k < Interactions.size(); k++) {
    if (Interactions[k].state == noContactState) {
      continue;
    }

    size_t i = Interactions[k].i;
    size_t j = Interactions[k].j;

    vec3r sij = Particles[j].pos - Particles[i].pos;
    sij.x -= floor(sij.x + 0.5);
    sij.y -= floor(sij.y + 0.5);
    sij.z -= floor(sij.z + 0.5);
    vec3r l = Cell.h * sij;
    vec3r n = l;
    n.normalize();
    // vec3r t = Interactions[k].ft;
    // t.normalize();

    // Noraml vector n
    nx = n.x;
    ny = n.y;
    nz = n.z;

    // vector t
    tx = ny / sqrt(nx * nx + ny * ny);
    ty = -nx / sqrt(nx * nx + ny * ny);
    tz = 0.0;

    // vector w
    wx = ny * tz - nz * ty;
    wy = nz * tx - nx * tz;
    wz = nx * ty - ny * tx;

    // xx
    L[x][x][x][x] +=
        kn_Vcell * nx * l.x * nx * l.x + kt_Vcell * tx * l.x * tx * l.x + kt_Vcell * wx * l.x * wx * l.x;  // 1
    // std::cout << "L[x][x][x][x]***" << L[x][x][x][x] << std::endl;
    L[x][x][y][y] +=
        kn_Vcell * nx * l.x * ny * l.y + kt_Vcell * tx * l.x * ty * l.y + kt_Vcell * wx * l.x * wy * l.y;  // 2
    L[x][x][z][z] +=
        kn_Vcell * nx * l.x * nz * l.z + kt_Vcell * tx * l.x * tz * l.z + kt_Vcell * wx * l.x * wz * l.z;  // 3
    L[x][x][x][y] +=
        kn_Vcell * nx * l.x * nx * l.y + kt_Vcell * tx * l.x * tx * l.y + kt_Vcell * wx * l.x * wx * l.y;  // 4
    L[x][x][x][z] +=
        kn_Vcell * nx * l.x * nx * l.z + kt_Vcell * tx * l.x * tx * l.z + kt_Vcell * wx * l.x * wx * l.z;  // 5
    L[x][x][y][z] +=
        kn_Vcell * nx * l.x * ny * l.z + kt_Vcell * tx * l.x * ty * l.z + kt_Vcell * wx * l.x * wy * l.z;  // 6
    L[x][x][y][x] +=
        kn_Vcell * nx * l.x * ny * l.x + kt_Vcell * tx * l.x * ty * l.x + kt_Vcell * wx * l.x * wy * l.x;  // 7
    L[x][x][z][x] +=
        kn_Vcell * nx * l.x * nz * l.x + kt_Vcell * tx * l.x * tz * l.x + kt_Vcell * wx * l.x * wz * l.x;  // 8
    L[x][x][z][y] +=
        kn_Vcell * nx * l.x * nz * l.y + kt_Vcell * tx * l.x * tz * l.y + kt_Vcell * wx * l.x * wz * l.y;  // 9

    // yy
    L[y][y][x][x] +=
        kn_Vcell * ny * l.y * nx * l.x + kt_Vcell * ty * l.y * tx * l.x + kt_Vcell * wy * l.y * wx * l.x;  // 1
    L[y][y][y][y] +=
        kn_Vcell * ny * l.y * ny * l.y + kt_Vcell * ty * l.y * ty * l.y + kt_Vcell * wy * l.y * wy * l.y;  // 2
    L[y][y][z][z] +=
        kn_Vcell * ny * l.y * nz * l.z + kt_Vcell * ty * l.y * tz * l.z + kt_Vcell * wy * l.y * wz * l.z;  // 3
    L[y][y][x][y] +=
        kn_Vcell * ny * l.y * nx * l.y + kt_Vcell * ty * l.y * tx * l.y + kt_Vcell * wy * l.y * wx * l.y;  // 4
    L[y][y][x][z] +=
        kn_Vcell * ny * l.y * nx * l.z + kt_Vcell * ty * l.y * tx * l.z + kt_Vcell * wy * l.y * wx * l.z;  // 5
    L[y][y][y][z] +=
        kn_Vcell * ny * l.y * ny * l.z + kt_Vcell * ty * l.y * ty * l.z + kt_Vcell * wy * l.y * wy * l.z;  // 6
    L[y][y][y][x] +=
        kn_Vcell * ny * l.y * ny * l.x + kt_Vcell * ty * l.y * ty * l.x + kt_Vcell * wy * l.y * wy * l.x;  // 7
    L[y][y][z][x] +=
        kn_Vcell * ny * l.y * nz * l.x + kt_Vcell * ty * l.y * tz * l.x + kt_Vcell * wy * l.y * wz * l.x;  // 8
    L[y][y][z][y] +=
        kn_Vcell * ny * l.y * nz * l.y + kt_Vcell * ty * l.y * tz * l.y + kt_Vcell * wy * l.y * wz * l.y;  // 9

    // zz
    L[z][z][x][x] +=
        kn_Vcell * nz * l.z * nx * l.x + kt_Vcell * tz * l.z * tx * l.x + kt_Vcell * wz * l.z * wx * l.x;  // 1
    L[z][z][y][y] +=
        kn_Vcell * nz * l.z * ny * l.y + kt_Vcell * tz * l.z * ty * l.y + kt_Vcell * wz * l.z * wy * l.y;  // 2
    L[z][z][z][z] +=
        kn_Vcell * nz * l.z * nz * l.z + kt_Vcell * tz * l.z * tz * l.z + kt_Vcell * wz * l.z * wz * l.z;  // 3
    L[z][z][x][y] +=
        kn_Vcell * nz * l.z * nx * l.y + kt_Vcell * tz * l.z * tx * l.y + kt_Vcell * wz * l.z * wx * l.y;  // 4
    L[z][z][x][z] +=
        kn_Vcell * nz * l.z * nx * l.z + kt_Vcell * tz * l.z * tx * l.z + kt_Vcell * wz * l.z * wx * l.z;  // 5
    L[z][z][y][z] +=
        kn_Vcell * nz * l.z * ny * l.z + kt_Vcell * tz * l.z * ty * l.z + kt_Vcell * wz * l.z * wy * l.z;  // 6
    L[z][z][y][x] +=
        kn_Vcell * nz * l.z * ny * l.x + kt_Vcell * tz * l.z * ty * l.x + kt_Vcell * wz * l.z * wy * l.x;  // 7
    L[z][z][z][x] +=
        kn_Vcell * nz * l.z * nz * l.x + kt_Vcell * tz * l.z * tz * l.x + kt_Vcell * wz * l.z * wz * l.x;  // 8
    L[z][z][z][y] +=
        kn_Vcell * nz * l.z * nz * l.y + kt_Vcell * tz * l.z * tz * l.y + kt_Vcell * wz * l.z * wz * l.y;  // 9

    // xy
    L[x][y][x][x] +=
        kn_Vcell * nx * l.y * nx * l.x + kt_Vcell * tx * l.y * tx * l.x + kt_Vcell * wx * l.y * wx * l.x;  // 1
    L[x][y][y][y] +=
        kn_Vcell * nx * l.y * ny * l.y + kt_Vcell * tx * l.y * ty * l.y + kt_Vcell * wx * l.y * wy * l.y;  // 2
    L[x][y][z][z] +=
        kn_Vcell * nx * l.y * nz * l.z + kt_Vcell * tx * l.y * tz * l.z + kt_Vcell * wx * l.y * wz * l.z;  // 3
    L[x][y][x][y] +=
        kn_Vcell * nx * l.y * nx * l.y + kt_Vcell * tx * l.y * tx * l.y + kt_Vcell * wx * l.y * wx * l.y;  // 4
    L[x][y][x][z] +=
        kn_Vcell * nx * l.y * nx * l.z + kt_Vcell * tx * l.y * tx * l.z + kt_Vcell * wx * l.y * wx * l.z;  // 5
    L[x][y][y][z] +=
        kn_Vcell * nx * l.y * ny * l.z + kt_Vcell * tx * l.y * ty * l.z + kt_Vcell * wx * l.y * wy * l.z;  // 6
    L[x][y][y][x] +=
        kn_Vcell * nx * l.y * ny * l.x + kt_Vcell * tx * l.y * ty * l.x + kt_Vcell * wx * l.y * wy * l.x;  // 7
    L[x][y][z][x] +=
        kn_Vcell * nx * l.y * nz * l.x + kt_Vcell * tx * l.y * tz * l.x + kt_Vcell * wx * l.y * wz * l.x;  // 8
    L[x][y][z][y] +=
        kn_Vcell * nx * l.y * nz * l.y + kt_Vcell * tx * l.y * tz * l.y + kt_Vcell * wx * l.y * wz * l.y;  // 9

    // xz
    L[x][z][x][x] +=
        kn_Vcell * nx * l.z * nx * l.x + kt_Vcell * tx * l.z * tx * l.x + kt_Vcell * wx * l.z * wx * l.x;  // 1
    L[x][z][y][y] +=
        kn_Vcell * nx * l.z * ny * l.y + kt_Vcell * tx * l.z * ty * l.y + kt_Vcell * wx * l.z * wy * l.y;  // 2
    L[x][z][z][z] +=
        kn_Vcell * nx * l.z * nz * l.z + kt_Vcell * tx * l.z * tz * l.z + kt_Vcell * wx * l.z * wz * l.z;  // 3
    L[x][z][x][y] +=
        kn_Vcell * nx * l.z * nx * l.y + kt_Vcell * tx * l.z * tx * l.y + kt_Vcell * wx * l.z * wx * l.y;  // 4
    L[x][z][x][z] +=
        kn_Vcell * nx * l.z * nx * l.z + kt_Vcell * tx * l.z * tx * l.z + kt_Vcell * wx * l.z * wx * l.z;  // 5
    L[x][z][y][z] +=
        kn_Vcell * nx * l.z * ny * l.z + kt_Vcell * tx * l.z * ty * l.z + kt_Vcell * wx * l.z * wy * l.z;  // 6
    L[x][z][y][x] +=
        kn_Vcell * nx * l.z * ny * l.x + kt_Vcell * tx * l.z * ty * l.x + kt_Vcell * wx * l.z * wy * l.x;  // 7
    L[x][z][z][x] +=
        kn_Vcell * nx * l.z * nz * l.x + kt_Vcell * tx * l.z * tz * l.x + kt_Vcell * wx * l.z * wz * l.x;  // 8
    L[x][z][z][y] +=
        kn_Vcell * nx * l.z * nz * l.y + kt_Vcell * tx * l.z * tz * l.y + kt_Vcell * wx * l.z * wz * l.y;  // 9

    // yz
    L[y][z][x][x] +=
        kn_Vcell * ny * l.z * nx * l.x + kt_Vcell * ty * l.z * tx * l.x + kt_Vcell * wy * l.z * wx * l.x;  // 1
    L[y][z][y][y] +=
        kn_Vcell * ny * l.z * ny * l.y + kt_Vcell * ty * l.z * ty * l.y + kt_Vcell * wy * l.z * wy * l.y;  // 2
    L[y][z][z][z] +=
        kn_Vcell * ny * l.z * nz * l.z + kt_Vcell * ty * l.z * tz * l.z + kt_Vcell * wy * l.z * wz * l.z;  // 3
    L[y][z][x][y] +=
        kn_Vcell * ny * l.z * nx * l.y + kt_Vcell * ty * l.z * tx * l.y + kt_Vcell * wy * l.z * wx * l.y;  // 4
    L[y][z][x][z] +=
        kn_Vcell * ny * l.z * nx * l.z + kt_Vcell * ty * l.z * tx * l.z + kt_Vcell * wy * l.z * wx * l.z;  // 5
    L[y][z][y][z] +=
        kn_Vcell * ny * l.z * ny * l.z + kt_Vcell * ty * l.z * ty * l.z + kt_Vcell * wy * l.z * wy * l.z;  // 6
    L[y][z][y][x] +=
        kn_Vcell * ny * l.z * ny * l.x + kt_Vcell * ty * l.z * ty * l.x + kt_Vcell * wy * l.z * wy * l.x;  // 7
    L[y][z][z][x] +=
        kn_Vcell * ny * l.z * nz * l.x + kt_Vcell * ty * l.z * tz * l.x + kt_Vcell * wy * l.z * wz * l.x;  // 8
    L[y][z][z][y] +=
        kn_Vcell * ny * l.z * nz * l.y + kt_Vcell * ty * l.z * tz * l.y + kt_Vcell * wy * l.z * wz * l.y;  // 9

    // yx
    L[y][x][x][x] +=
        kn_Vcell * ny * l.x * nx * l.x + kt_Vcell * ty * l.x * tx * l.x + kt_Vcell * wy * l.x * wx * l.x;  // 1
    L[y][x][y][y] +=
        kn_Vcell * ny * l.x * ny * l.y + kt_Vcell * ty * l.x * ty * l.y + kt_Vcell * wy * l.x * wy * l.y;  // 2
    L[y][x][z][z] +=
        kn_Vcell * ny * l.x * nz * l.z + kt_Vcell * ty * l.x * tz * l.z + kt_Vcell * wy * l.x * wz * l.z;  // 3
    L[y][x][x][y] +=
        kn_Vcell * ny * l.x * nx * l.y + kt_Vcell * ty * l.x * tx * l.y + kt_Vcell * wy * l.x * wx * l.y;  // 4
    L[y][x][x][z] +=
        kn_Vcell * ny * l.x * nx * l.z + kt_Vcell * ty * l.x * tx * l.z + kt_Vcell * wy * l.x * wx * l.z;  // 5
    L[y][x][y][z] +=
        kn_Vcell * ny * l.x * ny * l.z + kt_Vcell * ty * l.x * ty * l.z + kt_Vcell * wy * l.x * wy * l.z;  // 6
    L[y][x][y][x] +=
        kn_Vcell * ny * l.x * ny * l.x + kt_Vcell * ty * l.x * ty * l.x + kt_Vcell * wy * l.x * wy * l.x;  // 7
    L[y][x][z][x] +=
        kn_Vcell * ny * l.x * nz * l.x + kt_Vcell * ty * l.x * tz * l.x + kt_Vcell * wy * l.x * wz * l.x;  // 8
    L[y][x][z][y] +=
        kn_Vcell * ny * l.x * nz * l.y + kt_Vcell * ty * l.x * tz * l.y + kt_Vcell * wy * l.x * wz * l.y;  // 9

    // zx
    L[z][x][x][x] +=
        kn_Vcell * nz * l.x * nx * l.x + kt_Vcell * tz * l.x * tx * l.x + kt_Vcell * wz * l.x * wx * l.x;  // 1
    L[z][x][y][y] +=
        kn_Vcell * nz * l.x * ny * l.y + kt_Vcell * tz * l.x * ty * l.y + kt_Vcell * wz * l.x * wy * l.y;  // 2
    L[z][x][z][z] +=
        kn_Vcell * nz * l.x * nz * l.z + kt_Vcell * tz * l.x * tz * l.z + kt_Vcell * wz * l.x * wz * l.z;  // 3
    L[z][x][x][y] +=
        kn_Vcell * nz * l.x * nx * l.y + kt_Vcell * tz * l.x * tx * l.y + kt_Vcell * wz * l.x * wx * l.y;  // 4
    L[z][x][x][z] +=
        kn_Vcell * nz * l.x * nx * l.z + kt_Vcell * tz * l.x * tx * l.z + kt_Vcell * wz * l.x * wx * l.z;  // 5
    L[z][x][y][z] +=
        kn_Vcell * nz * l.x * ny * l.z + kt_Vcell * tz * l.x * ty * l.z + kt_Vcell * wz * l.x * wy * l.z;  // 6
    L[z][x][y][x] +=
        kn_Vcell * nz * l.x * ny * l.x + kt_Vcell * tz * l.x * ty * l.x + kt_Vcell * wz * l.x * wy * l.x;  // 7
    L[z][x][z][x] +=
        kn_Vcell * nz * l.x * nz * l.x + kt_Vcell * tz * l.x * tz * l.x + kt_Vcell * wz * l.x * wz * l.x;  // 8
    L[z][x][z][y] +=
        kn_Vcell * nz * l.x * nz * l.y + kt_Vcell * tz * l.x * tz * l.y + kt_Vcell * wz * l.x * wz * l.y;  // 9

    // zy
    L[z][y][x][x] +=
        kn_Vcell * nz * l.y * nx * l.x + kt_Vcell * tz * l.y * tx * l.x + kt_Vcell * wz * l.y * wx * l.x;  // 1
    L[z][y][y][y] +=
        kn_Vcell * nz * l.y * ny * l.y + kt_Vcell * tz * l.y * ty * l.y + kt_Vcell * wz * l.y * wy * l.y;  // 2
    L[z][y][z][z] +=
        kn_Vcell * nz * l.y * nz * l.z + kt_Vcell * tz * l.y * tz * l.z + kt_Vcell * wz * l.y * wz * l.z;  // 3
    L[z][y][x][y] +=
        kn_Vcell * nz * l.y * nx * l.y + kt_Vcell * tz * l.y * tx * l.y + kt_Vcell * wz * l.y * wx * l.y;  // 4
    L[z][y][x][z] +=
        kn_Vcell * nz * l.y * nx * l.z + kt_Vcell * tz * l.y * tx * l.z + kt_Vcell * wz * l.y * wx * l.z;  // 5
    L[z][y][y][z] +=
        kn_Vcell * nz * l.y * ny * l.z + kt_Vcell * tz * l.y * ty * l.z + kt_Vcell * wz * l.y * wy * l.z;  // 6
    L[z][y][y][x] +=
        kn_Vcell * nz * l.y * ny * l.x + kt_Vcell * tz * l.y * ty * l.x + kt_Vcell * wz * l.y * wy * l.x;  // 7
    L[z][y][z][x] +=
        kn_Vcell * nz * l.y * nz * l.x + kt_Vcell * tz * l.y * tz * l.x + kt_Vcell * wz * l.y * wz * l.x;  // 8
    L[z][y][z][y] +=
        kn_Vcell * nz * l.y * nz * l.y + kt_Vcell * tz * l.y * tz * l.y + kt_Vcell * wz * l.y * wz * l.y;  // 9
  }
}

// This 3rd version takes care of the sliding contacts
void PBC3Dbox::getOperatorKruyt3(double L[9][9]) {
  for (size_t i = 0; i < 9; i++) {
    for (size_t j = 0; j < 9; j++) {
      L[i][j] = 0.0;
    }
  }

  // precomputations
  double Vcell = fabs(Cell.h.det());
  double kn_Vcell = 0.93 * kn / Vcell;
  double kt_Vcell = 0.63 * kt / Vcell;

  // indexes
  const size_t xx = 0;
  const size_t yy = 1;
  const size_t zz = 2;
  const size_t xy = 3;
  const size_t xz = 4;
  const size_t yz = 5;
  const size_t yx = 6;
  const size_t zx = 7;
  const size_t zy = 8;

  double nx, ny, nz, tx, ty, tz, wx, wy, wz;
  double Lxxxx, Lxxyy, Lxxzz, Lxxxy, Lxxxz, Lxxyz, Lxxyx, Lxxzx, Lxxzy;
  double Lyyxx, Lyyyy, Lyyzz, Lyyxy, Lyyxz, Lyyyz, Lyyyx, Lyyzx, Lyyzy;
  double Lzzxx, Lzzyy, Lzzzz, Lzzxy, Lzzxz, Lzzyz, Lzzyx, Lzzzx, Lzzzy;
  double Lxyxx, Lxyyy, Lxyzz, Lxyxy, Lxyxz, Lxyyz, Lxyyx, Lxyzx, Lxyzy;
  double Lxzxx, Lxzyy, Lxzzz, Lxzxy, Lxzxz, Lxzyz, Lxzyx, Lxzzx, Lxzzy;
  double Lyzxx, Lyzyy, Lyzzz, Lyzxy, Lyzxz, Lyzyz, Lyzyx, Lyzzx, Lyzzy;
  double Lyxxx, Lyxyy, Lyxzz, Lyxxy, Lyxxz, Lyxyz, Lyxyx, Lyxzx, Lyxzy;
  double Lzxxx, Lzxyy, Lzxzz, Lzxxy, Lzxxz, Lzxyz, Lzxyx, Lzxzx, Lzxzy;
  double Lzyxx, Lzyyy, Lzyzz, Lzyxy, Lzyxz, Lzyyz, Lzyyx, Lzyzx, Lzyzy;

  double Lknxxxx, Lktxxxx, Lknxxyy, Lktxxyy;
  double Lkn1 = 0.0;
  double Lkn2 = 0.0;
  double Lkt1 = 0.0;
  double Lkt2 = 0.0;

  for (size_t k = 0; k < Interactions.size(); k++) {
    if (Interactions[k].state == noContactState) {
      continue;
    }

    size_t i = Interactions[k].i;
    size_t j = Interactions[k].j;

    vec3r sij = Particles[j].pos - Particles[i].pos;
    sij.x -= floor(sij.x + 0.5);
    sij.y -= floor(sij.y + 0.5);
    sij.z -= floor(sij.z + 0.5);
    vec3r l = Cell.h * sij;
    vec3r n = l;
    n.normalize();
    // vec3r t = Interactions[k].ft;
    // t.normalize();

    // vector normal n
    nx = n.x;
    ny = n.y;
    nz = n.z;

    // vector t
    tx = ny / sqrt(nx * nx + ny * ny);
    ty = -nx / sqrt(nx * nx + ny * ny);
    tz = 0.0;

    // vector w
    wx = ny * tz - nz * ty;
    wy = nz * tx - nx * tz;
    wz = nx * ty - ny * tx;

    // Verification: sliding or not?
    double threshold = fabs(mu * Interactions[k].fn);  // Suppose that fn is elastic and without cohesion
    double ft_square = Interactions[k].ft * Interactions[k].ft;
    double tol = threshold * threshold - ft_square;

    if (ft_square > 0.0 && sqrt(tol) <= 1e-15) {  // sliding contact

      // xx
      Lxxxx = kn_Vcell * nx * l.x * nx * l.x + mu * kn_Vcell * tx * l.x * wx * l.x + kt_Vcell * wx * l.x * wx * l.x;
      Lknxxxx = kn_Vcell * nx * l.x * nx * l.x + mu * kn_Vcell * tx * l.x * wx * l.x;
      Lktxxxx = kt_Vcell * wx * l.x * wx * l.x;

      Lxxyy = kn_Vcell * nx * l.x * ny * l.y + mu * kn_Vcell * tx * l.x * wy * l.y + kt_Vcell * wx * l.x * wy * l.y;
      Lknxxyy = kn_Vcell * nx * l.x * ny * l.y + mu * kn_Vcell * tx * l.x * wy * l.y;
      Lktxxyy = kt_Vcell * wx * l.x * wy * l.y;

      Lxxzz = kn_Vcell * nx * l.x * nz * l.z + mu * kn_Vcell * tx * l.x * wz * l.z + kt_Vcell * wx * l.x * wz * l.z;
      Lxxxy = kn_Vcell * nx * l.x * nx * l.y + mu * kn_Vcell * tx * l.x * wx * l.y + kt_Vcell * wx * l.x * wx * l.y;
      Lxxxz = kn_Vcell * nx * l.x * nx * l.z + mu * kn_Vcell * tx * l.x * wx * l.z + kt_Vcell * wx * l.x * wx * l.z;
      Lxxyz = kn_Vcell * nx * l.x * ny * l.z + mu * kn_Vcell * tx * l.x * wy * l.z + kt_Vcell * wx * l.x * wy * l.z;
      Lxxyx = kn_Vcell * nx * l.x * ny * l.x + mu * kn_Vcell * tx * l.x * wy * l.x + kt_Vcell * wx * l.x * wy * l.x;
      Lxxzx = kn_Vcell * nx * l.x * nz * l.x + mu * kn_Vcell * tx * l.x * wz * l.x + kt_Vcell * wx * l.x * wz * l.x;
      Lxxzy = kn_Vcell * nx * l.x * nz * l.y + mu * kn_Vcell * tx * l.x * wz * l.y + kt_Vcell * wx * l.x * wz * l.y;

      // yy
      Lyyxx = kn_Vcell * ny * l.y * nx * l.x + mu * kn_Vcell * ty * l.y * wx * l.x + kt_Vcell * wy * l.y * wx * l.x;
      Lyyyy = kn_Vcell * ny * l.y * ny * l.y + mu * kn_Vcell * ty * l.y * wy * l.y + kt_Vcell * wy * l.y * wy * l.y;
      Lyyzz = kn_Vcell * ny * l.y * nz * l.z + mu * kn_Vcell * ty * l.y * wz * l.z + kt_Vcell * wy * l.y * wz * l.z;
      Lyyxy = kn_Vcell * ny * l.y * nx * l.y + mu * kn_Vcell * ty * l.y * wx * l.y + kt_Vcell * wy * l.y * wx * l.y;
      Lyyxz = kn_Vcell * ny * l.y * nx * l.z + mu * kn_Vcell * ty * l.y * wx * l.z + kt_Vcell * wy * l.y * wx * l.z;
      Lyyyz = kn_Vcell * ny * l.y * ny * l.z + mu * kn_Vcell * ty * l.y * wy * l.z + kt_Vcell * wy * l.y * wy * l.z;
      Lyyyx = kn_Vcell * ny * l.y * ny * l.x + mu * kn_Vcell * ty * l.y * wy * l.x + kt_Vcell * wy * l.y * wy * l.x;
      Lyyzx = kn_Vcell * ny * l.y * nz * l.x + mu * kn_Vcell * ty * l.y * wz * l.x + kt_Vcell * wy * l.y * wz * l.x;
      Lyyzy = kn_Vcell * ny * l.y * nz * l.y + mu * kn_Vcell * ty * l.y * wz * l.y + kt_Vcell * wy * l.y * wz * l.y;

      // zz
      Lzzxx = kn_Vcell * nz * l.z * nx * l.x + mu * kn_Vcell * tz * l.z * wx * l.x + kt_Vcell * wz * l.z * wx * l.x;
      Lzzyy = kn_Vcell * nz * l.z * ny * l.y + mu * kn_Vcell * tz * l.z * wy * l.y + kt_Vcell * wz * l.z * wy * l.y;
      Lzzzz = kn_Vcell * nz * l.z * nz * l.z + mu * kn_Vcell * tz * l.z * wz * l.z + kt_Vcell * wz * l.z * wz * l.z;
      Lzzxy = kn_Vcell * nz * l.z * nx * l.y + mu * kn_Vcell * tz * l.z * wx * l.y + kt_Vcell * wz * l.z * wx * l.y;
      Lzzxz = kn_Vcell * nz * l.z * nx * l.z + mu * kn_Vcell * tz * l.z * wx * l.z + kt_Vcell * wz * l.z * wx * l.z;
      Lzzyz = kn_Vcell * nz * l.z * ny * l.z + mu * kn_Vcell * tz * l.z * wy * l.z + kt_Vcell * wz * l.z * wy * l.z;
      Lzzyx = kn_Vcell * nz * l.z * ny * l.x + mu * kn_Vcell * tz * l.z * wy * l.x + kt_Vcell * wz * l.z * wy * l.x;
      Lzzzx = kn_Vcell * nz * l.z * nz * l.x + mu * kn_Vcell * tz * l.z * wz * l.x + kt_Vcell * wz * l.z * wz * l.x;
      Lzzzy = kn_Vcell * nz * l.z * nz * l.y + mu * kn_Vcell * tz * l.z * wz * l.y + kt_Vcell * wz * l.z * wz * l.y;

      // xy
      Lxyxx = kn_Vcell * nx * l.y * nx * l.x + mu * kn_Vcell * tx * l.y * wx * l.x + kt_Vcell * wx * l.y * wx * l.x;
      Lxyyy = kn_Vcell * nx * l.y * ny * l.y + mu * kn_Vcell * tx * l.y * wy * l.y + kt_Vcell * wx * l.y * wy * l.y;
      Lxyzz = kn_Vcell * nx * l.y * nz * l.z + mu * kn_Vcell * tx * l.y * wz * l.z + kt_Vcell * wx * l.y * wz * l.z;
      Lxyxy = kn_Vcell * nx * l.y * nx * l.y + mu * kn_Vcell * tx * l.y * wx * l.y + kt_Vcell * wx * l.y * wx * l.y;
      Lxyxz = kn_Vcell * nx * l.y * nx * l.z + mu * kn_Vcell * tx * l.y * wx * l.z + kt_Vcell * wx * l.y * wx * l.z;
      Lxyyz = kn_Vcell * nx * l.y * ny * l.z + mu * kn_Vcell * tx * l.y * wy * l.z + kt_Vcell * wx * l.y * wy * l.z;
      Lxyyx = kn_Vcell * nx * l.y * ny * l.x + mu * kn_Vcell * tx * l.y * wy * l.x + kt_Vcell * wx * l.y * wy * l.x;
      Lxyzx = kn_Vcell * nx * l.y * nz * l.x + mu * kn_Vcell * tx * l.y * wz * l.x + kt_Vcell * wx * l.y * wz * l.x;
      Lxyzy = kn_Vcell * nx * l.y * nz * l.y + mu * kn_Vcell * tx * l.y * wz * l.y + kt_Vcell * wx * l.y * wz * l.y;

      // xz
      Lxzxx = kn_Vcell * nx * l.z * nx * l.x + mu * kn_Vcell * tx * l.z * wx * l.x + kt_Vcell * wx * l.z * wx * l.x;
      Lxzyy = kn_Vcell * nx * l.z * ny * l.y + mu * kn_Vcell * tx * l.z * wy * l.y + kt_Vcell * wx * l.z * wy * l.y;
      Lxzzz = kn_Vcell * nx * l.z * nz * l.z + mu * kn_Vcell * tx * l.z * wz * l.z + kt_Vcell * wx * l.z * wz * l.z;
      Lxzxy = kn_Vcell * nx * l.z * nx * l.y + mu * kn_Vcell * tx * l.z * wx * l.y + kt_Vcell * wx * l.z * wx * l.y;
      Lxzxz = kn_Vcell * nx * l.z * nx * l.z + mu * kn_Vcell * tx * l.z * wx * l.z + kt_Vcell * wx * l.z * wx * l.z;
      Lxzyz = kn_Vcell * nx * l.z * ny * l.z + mu * kn_Vcell * tx * l.z * wy * l.z + kt_Vcell * wx * l.z * wy * l.z;
      Lxzyx = kn_Vcell * nx * l.z * ny * l.x + mu * kn_Vcell * tx * l.z * wy * l.x + kt_Vcell * wx * l.z * wy * l.x;
      Lxzzx = kn_Vcell * nx * l.z * nz * l.x + mu * kn_Vcell * tx * l.z * wz * l.x + kt_Vcell * wx * l.z * wz * l.x;
      Lxzzy = kn_Vcell * nx * l.z * nz * l.y + mu * kn_Vcell * tx * l.z * wz * l.y + kt_Vcell * wx * l.z * wz * l.y;

      // yz
      Lyzxx = kn_Vcell * ny * l.z * nx * l.x + mu * kn_Vcell * ty * l.z * wx * l.x + kt_Vcell * wy * l.z * wx * l.x;
      Lyzyy = kn_Vcell * ny * l.z * ny * l.y + mu * kn_Vcell * ty * l.z * wy * l.y + kt_Vcell * wy * l.z * wy * l.y;
      Lyzzz = kn_Vcell * ny * l.z * nz * l.z + mu * kn_Vcell * ty * l.z * wz * l.z + kt_Vcell * wy * l.z * wz * l.z;
      Lyzxy = kn_Vcell * ny * l.z * nx * l.y + mu * kn_Vcell * ty * l.z * wx * l.y + kt_Vcell * wy * l.z * wx * l.y;
      Lyzxz = kn_Vcell * ny * l.z * nx * l.z + mu * kn_Vcell * ty * l.z * wx * l.z + kt_Vcell * wy * l.z * wx * l.z;
      Lyzyz = kn_Vcell * ny * l.z * ny * l.z + mu * kn_Vcell * ty * l.z * wy * l.z + kt_Vcell * wy * l.z * wy * l.z;
      Lyzyx = kn_Vcell * ny * l.z * ny * l.x + mu * kn_Vcell * ty * l.z * wy * l.x + kt_Vcell * wy * l.z * wy * l.x;
      Lyzzx = kn_Vcell * ny * l.z * nz * l.x + mu * kn_Vcell * ty * l.z * wz * l.x + kt_Vcell * wy * l.z * wz * l.x;
      Lyzzy = kn_Vcell * ny * l.z * nz * l.y + mu * kn_Vcell * ty * l.z * wz * l.y + kt_Vcell * wy * l.z * wz * l.y;

      // yx
      Lyxxx = kn_Vcell * ny * l.x * nx * l.x + mu * kn_Vcell * ty * l.x * wx * l.x + kt_Vcell * wy * l.x * wx * l.x;
      Lyxyy = kn_Vcell * ny * l.x * ny * l.y + mu * kn_Vcell * ty * l.x * wy * l.y + kt_Vcell * wy * l.x * wy * l.y;
      Lyxzz = kn_Vcell * ny * l.x * nz * l.z + mu * kn_Vcell * ty * l.x * wz * l.z + kt_Vcell * wy * l.x * wz * l.z;
      Lyxxy = kn_Vcell * ny * l.x * nx * l.y + mu * kn_Vcell * ty * l.x * wx * l.y + kt_Vcell * wy * l.x * wx * l.y;
      Lyxxz = kn_Vcell * ny * l.x * nx * l.z + mu * kn_Vcell * ty * l.x * wx * l.z + kt_Vcell * wy * l.x * wx * l.z;
      Lyxyz = kn_Vcell * ny * l.x * ny * l.z + mu * kn_Vcell * ty * l.x * wy * l.z + kt_Vcell * wy * l.x * wy * l.z;
      Lyxyx = kn_Vcell * ny * l.x * ny * l.x + mu * kn_Vcell * ty * l.x * wy * l.x + kt_Vcell * wy * l.x * wy * l.x;
      Lyxzx = kn_Vcell * ny * l.x * nz * l.x + mu * kn_Vcell * ty * l.x * wz * l.x + kt_Vcell * wy * l.x * wz * l.x;
      Lyxzy = kn_Vcell * ny * l.x * nz * l.y + mu * kn_Vcell * ty * l.x * wz * l.y + kt_Vcell * wy * l.x * wz * l.y;

      // zx
      Lzxxx = kn_Vcell * nz * l.x * nx * l.x + mu * kn_Vcell * tz * l.x * wx * l.x + kt_Vcell * wz * l.x * wx * l.x;
      Lzxyy = kn_Vcell * nz * l.x * ny * l.y + mu * kn_Vcell * tz * l.x * wy * l.y + kt_Vcell * wz * l.x * wy * l.y;
      Lzxzz = kn_Vcell * nz * l.x * nz * l.z + mu * kn_Vcell * tz * l.x * wz * l.z + kt_Vcell * wz * l.x * wz * l.z;
      Lzxxy = kn_Vcell * nz * l.x * nx * l.y + mu * kn_Vcell * tz * l.x * wx * l.y + kt_Vcell * wz * l.x * wx * l.y;
      Lzxxz = kn_Vcell * nz * l.x * nx * l.z + mu * kn_Vcell * tz * l.x * wx * l.z + kt_Vcell * wz * l.x * wx * l.z;
      Lzxyz = kn_Vcell * nz * l.x * ny * l.z + mu * kn_Vcell * tz * l.x * wy * l.z + kt_Vcell * wz * l.x * wy * l.z;
      Lzxyx = kn_Vcell * nz * l.x * ny * l.x + mu * kn_Vcell * tz * l.x * wy * l.x + kt_Vcell * wz * l.x * wy * l.x;
      Lzxzx = kn_Vcell * nz * l.x * nz * l.x + mu * kn_Vcell * tz * l.x * wz * l.x + kt_Vcell * wz * l.x * wz * l.x;
      Lzxzy = kn_Vcell * nz * l.x * nz * l.y + mu * kn_Vcell * tz * l.x * wz * l.y + kt_Vcell * wz * l.x * wz * l.y;

      // zy
      Lzyxx = kn_Vcell * nz * l.y * nx * l.x + mu * kn_Vcell * tz * l.y * wx * l.x + kt_Vcell * wz * l.y * wx * l.x;
      Lzyyy = kn_Vcell * nz * l.y * ny * l.y + mu * kn_Vcell * tz * l.y * wy * l.y + kt_Vcell * wz * l.y * wy * l.y;
      Lzyzz = kn_Vcell * nz * l.y * nz * l.z + mu * kn_Vcell * tz * l.y * wz * l.z + kt_Vcell * wz * l.y * wz * l.z;
      Lzyxy = kn_Vcell * nz * l.y * nx * l.y + mu * kn_Vcell * tz * l.y * wx * l.y + kt_Vcell * wz * l.y * wx * l.y;
      Lzyxz = kn_Vcell * nz * l.y * nx * l.z + mu * kn_Vcell * tz * l.y * wx * l.z + kt_Vcell * wz * l.y * wx * l.z;
      Lzyyz = kn_Vcell * nz * l.y * ny * l.z + mu * kn_Vcell * tz * l.y * wy * l.z + kt_Vcell * wz * l.y * wy * l.z;
      Lzyyx = kn_Vcell * nz * l.y * ny * l.x + mu * kn_Vcell * tz * l.y * wy * l.x + kt_Vcell * wz * l.y * wy * l.x;
      Lzyzx = kn_Vcell * nz * l.y * nz * l.x + mu * kn_Vcell * tz * l.y * wz * l.x + kt_Vcell * wz * l.y * wz * l.x;
      Lzyzy = kn_Vcell * nz * l.y * nz * l.y + mu * kn_Vcell * tz * l.y * wz * l.y + kt_Vcell * wz * l.y * wz * l.y;
    } else {  // no sliding contact
      // xx
      Lxxxx = kn_Vcell * nx * l.x * nx * l.x + kt_Vcell * tx * l.x * tx * l.x + kt_Vcell * wx * l.x * wx * l.x;  // 1
      Lknxxxx = kn_Vcell * nx * l.x * nx * l.x;
      Lktxxxx = kt_Vcell * wx * l.x * wx * l.x + kt_Vcell * tx * l.x * tx * l.x;
      Lxxyy = kn_Vcell * nx * l.x * ny * l.y + kt_Vcell * tx * l.x * ty * l.y + kt_Vcell * wx * l.x * wy * l.y;  // 2
      Lknxxyy = kn_Vcell * nx * l.x * ny * l.y;
      Lktxxyy = kt_Vcell * wx * l.x * wy * l.y + kt_Vcell * tx * l.x * ty * l.y;

      Lxxzz = kn_Vcell * nx * l.x * nz * l.z + kt_Vcell * tx * l.x * tz * l.z + kt_Vcell * wx * l.x * wz * l.z;  // 3
      Lxxxy = kn_Vcell * nx * l.x * nx * l.y + kt_Vcell * tx * l.x * tx * l.y + kt_Vcell * wx * l.x * wx * l.y;  // 4
      Lxxxz = kn_Vcell * nx * l.x * nx * l.z + kt_Vcell * tx * l.x * tx * l.z + kt_Vcell * wx * l.x * wx * l.z;  // 5
      Lxxyz = kn_Vcell * nx * l.x * ny * l.z + kt_Vcell * tx * l.x * ty * l.z + kt_Vcell * wx * l.x * wy * l.z;  // 6
      Lxxyx = kn_Vcell * nx * l.x * ny * l.x + kt_Vcell * tx * l.x * ty * l.x + kt_Vcell * wx * l.x * wy * l.x;  // 7
      Lxxzx = kn_Vcell * nx * l.x * nz * l.x + kt_Vcell * tx * l.x * tz * l.x + kt_Vcell * wx * l.x * wz * l.x;  // 8
      Lxxzy = kn_Vcell * nx * l.x * nz * l.y + kt_Vcell * tx * l.x * tz * l.y + kt_Vcell * wx * l.x * wz * l.y;  // 9

      // yy
      Lyyxx = kn_Vcell * ny * l.y * nx * l.x + kt_Vcell * ty * l.y * tx * l.x + kt_Vcell * wy * l.y * wx * l.x;  // 1
      Lyyyy = kn_Vcell * ny * l.y * ny * l.y + kt_Vcell * ty * l.y * ty * l.y + kt_Vcell * wy * l.y * wy * l.y;  // 2
      Lyyzz = kn_Vcell * ny * l.y * nz * l.z + kt_Vcell * ty * l.y * tz * l.z + kt_Vcell * wy * l.y * wz * l.z;  // 3
      Lyyxy = kn_Vcell * ny * l.y * nx * l.y + kt_Vcell * ty * l.y * tx * l.y + kt_Vcell * wy * l.y * wx * l.y;  // 4
      Lyyxz = kn_Vcell * ny * l.y * nx * l.z + kt_Vcell * ty * l.y * tx * l.z + kt_Vcell * wy * l.y * wx * l.z;  // 5
      Lyyyz = kn_Vcell * ny * l.y * ny * l.z + kt_Vcell * ty * l.y * ty * l.z + kt_Vcell * wy * l.y * wy * l.z;  // 6
      Lyyyx = kn_Vcell * ny * l.y * ny * l.x + kt_Vcell * ty * l.y * ty * l.x + kt_Vcell * wy * l.y * wy * l.x;  // 7
      Lyyzx = kn_Vcell * ny * l.y * nz * l.x + kt_Vcell * ty * l.y * tz * l.x + kt_Vcell * wy * l.y * wz * l.x;  // 8
      Lyyzy = kn_Vcell * ny * l.y * nz * l.y + kt_Vcell * ty * l.y * tz * l.y + kt_Vcell * wy * l.y * wz * l.y;  // 9

      // zz
      Lzzxx = kn_Vcell * nz * l.z * nx * l.x + kt_Vcell * tz * l.z * tx * l.x + kt_Vcell * wz * l.z * wx * l.x;  // 1
      Lzzyy = kn_Vcell * nz * l.z * ny * l.y + kt_Vcell * tz * l.z * ty * l.y + kt_Vcell * wz * l.z * wy * l.y;  // 2
      Lzzzz = kn_Vcell * nz * l.z * nz * l.z + kt_Vcell * tz * l.z * tz * l.z + kt_Vcell * wz * l.z * wz * l.z;  // 3
      Lzzxy = kn_Vcell * nz * l.z * nx * l.y + kt_Vcell * tz * l.z * tx * l.y + kt_Vcell * wz * l.z * wx * l.y;  // 4
      Lzzxz = kn_Vcell * nz * l.z * nx * l.z + kt_Vcell * tz * l.z * tx * l.z + kt_Vcell * wz * l.z * wx * l.z;  // 5
      Lzzyz = kn_Vcell * nz * l.z * ny * l.z + kt_Vcell * tz * l.z * ty * l.z + kt_Vcell * wz * l.z * wy * l.z;  // 6
      Lzzyx = kn_Vcell * nz * l.z * ny * l.x + kt_Vcell * tz * l.z * ty * l.x + kt_Vcell * wz * l.z * wy * l.x;  // 7
      Lzzzx = kn_Vcell * nz * l.z * nz * l.x + kt_Vcell * tz * l.z * tz * l.x + kt_Vcell * wz * l.z * wz * l.x;  // 8
      Lzzzy = kn_Vcell * nz * l.z * nz * l.y + kt_Vcell * tz * l.z * tz * l.y + kt_Vcell * wz * l.z * wz * l.y;  // 9

      // xy
      Lxyxx = kn_Vcell * nx * l.y * nx * l.x + kt_Vcell * tx * l.y * tx * l.x + kt_Vcell * wx * l.y * wx * l.x;  // 1
      Lxyyy = kn_Vcell * nx * l.y * ny * l.y + kt_Vcell * tx * l.y * ty * l.y + kt_Vcell * wx * l.y * wy * l.y;  // 2
      Lxyzz = kn_Vcell * nx * l.y * nz * l.z + kt_Vcell * tx * l.y * tz * l.z + kt_Vcell * wx * l.y * wz * l.z;  // 3
      Lxyxy = kn_Vcell * nx * l.y * nx * l.y + kt_Vcell * tx * l.y * tx * l.y + kt_Vcell * wx * l.y * wx * l.y;  // 4
      Lxyxz = kn_Vcell * nx * l.y * nx * l.z + kt_Vcell * tx * l.y * tx * l.z + kt_Vcell * wx * l.y * wx * l.z;  // 5
      Lxyyz = kn_Vcell * nx * l.y * ny * l.z + kt_Vcell * tx * l.y * ty * l.z + kt_Vcell * wx * l.y * wy * l.z;  // 6
      Lxyyx = kn_Vcell * nx * l.y * ny * l.x + kt_Vcell * tx * l.y * ty * l.x + kt_Vcell * wx * l.y * wy * l.x;  // 7
      Lxyzx = kn_Vcell * nx * l.y * nz * l.x + kt_Vcell * tx * l.y * tz * l.x + kt_Vcell * wx * l.y * wz * l.x;  // 8
      Lxyzy = kn_Vcell * nx * l.y * nz * l.y + kt_Vcell * tx * l.y * tz * l.y + kt_Vcell * wx * l.y * wz * l.y;  // 9

      // xz
      Lxzxx = kn_Vcell * nx * l.z * nx * l.x + kt_Vcell * tx * l.z * tx * l.x + kt_Vcell * wx * l.z * wx * l.x;  // 1
      Lxzyy = kn_Vcell * nx * l.z * ny * l.y + kt_Vcell * tx * l.z * ty * l.y + kt_Vcell * wx * l.z * wy * l.y;  // 2
      Lxzzz = kn_Vcell * nx * l.z * nz * l.z + kt_Vcell * tx * l.z * tz * l.z + kt_Vcell * wx * l.z * wz * l.z;  // 3
      Lxzxy = kn_Vcell * nx * l.z * nx * l.y + kt_Vcell * tx * l.z * tx * l.y + kt_Vcell * wx * l.z * wx * l.y;  // 4
      Lxzxz = kn_Vcell * nx * l.z * nx * l.z + kt_Vcell * tx * l.z * tx * l.z + kt_Vcell * wx * l.z * wx * l.z;  // 5
      Lxzyz = kn_Vcell * nx * l.z * ny * l.z + kt_Vcell * tx * l.z * ty * l.z + kt_Vcell * wx * l.z * wy * l.z;  // 6
      Lxzyx = kn_Vcell * nx * l.z * ny * l.x + kt_Vcell * tx * l.z * ty * l.x + kt_Vcell * wx * l.z * wy * l.x;  // 7
      Lxzzx = kn_Vcell * nx * l.z * nz * l.x + kt_Vcell * tx * l.z * tz * l.x + kt_Vcell * wx * l.z * wz * l.x;  // 8
      Lxzzy = kn_Vcell * nx * l.z * nz * l.y + kt_Vcell * tx * l.z * tz * l.y + kt_Vcell * wx * l.z * wz * l.y;  // 9

      // yz
      Lyzxx = kn_Vcell * ny * l.z * nx * l.x + kt_Vcell * ty * l.z * tx * l.x + kt_Vcell * wy * l.z * wx * l.x;  // 1
      Lyzyy = kn_Vcell * ny * l.z * ny * l.y + kt_Vcell * ty * l.z * ty * l.y + kt_Vcell * wy * l.z * wy * l.y;  // 2
      Lyzzz = kn_Vcell * ny * l.z * nz * l.z + kt_Vcell * ty * l.z * tz * l.z + kt_Vcell * wy * l.z * wz * l.z;  // 3
      Lyzxy = kn_Vcell * ny * l.z * nx * l.y + kt_Vcell * ty * l.z * tx * l.y + kt_Vcell * wy * l.z * wx * l.y;  // 4
      Lyzxz = kn_Vcell * ny * l.z * nx * l.z + kt_Vcell * ty * l.z * tx * l.z + kt_Vcell * wy * l.z * wx * l.z;  // 5
      Lyzyz = kn_Vcell * ny * l.z * ny * l.z + kt_Vcell * ty * l.z * ty * l.z + kt_Vcell * wy * l.z * wy * l.z;  // 6
      Lyzyx = kn_Vcell * ny * l.z * ny * l.x + kt_Vcell * ty * l.z * ty * l.x + kt_Vcell * wy * l.z * wy * l.x;  // 7
      Lyzzx = kn_Vcell * ny * l.z * nz * l.x + kt_Vcell * ty * l.z * tz * l.x + kt_Vcell * wy * l.z * wz * l.x;  // 8
      Lyzzy = kn_Vcell * ny * l.z * nz * l.y + kt_Vcell * ty * l.z * tz * l.y + kt_Vcell * wy * l.z * wz * l.y;  // 9

      // yx
      Lyxxx = kn_Vcell * ny * l.x * nx * l.x + kt_Vcell * ty * l.x * tx * l.x + kt_Vcell * wy * l.x * wx * l.x;  // 1
      Lyxyy = kn_Vcell * ny * l.x * ny * l.y + kt_Vcell * ty * l.x * ty * l.y + kt_Vcell * wy * l.x * wy * l.y;  // 2
      Lyxzz = kn_Vcell * ny * l.x * nz * l.z + kt_Vcell * ty * l.x * tz * l.z + kt_Vcell * wy * l.x * wz * l.z;  // 3
      Lyxxy = kn_Vcell * ny * l.x * nx * l.y + kt_Vcell * ty * l.x * tx * l.y + kt_Vcell * wy * l.x * wx * l.y;  // 4
      Lyxxz = kn_Vcell * ny * l.x * nx * l.z + kt_Vcell * ty * l.x * tx * l.z + kt_Vcell * wy * l.x * wx * l.z;  // 5
      Lyxyz = kn_Vcell * ny * l.x * ny * l.z + kt_Vcell * ty * l.x * ty * l.z + kt_Vcell * wy * l.x * wy * l.z;  // 6
      Lyxyx = kn_Vcell * ny * l.x * ny * l.x + kt_Vcell * ty * l.x * ty * l.x + kt_Vcell * wy * l.x * wy * l.x;  // 7
      Lyxzx = kn_Vcell * ny * l.x * nz * l.x + kt_Vcell * ty * l.x * tz * l.x + kt_Vcell * wy * l.x * wz * l.x;  // 8
      Lyxzy = kn_Vcell * ny * l.x * nz * l.y + kt_Vcell * ty * l.x * tz * l.y + kt_Vcell * wy * l.x * wz * l.y;  // 9

      // zx
      Lzxxx = kn_Vcell * nz * l.x * nx * l.x + kt_Vcell * tz * l.x * tx * l.x + kt_Vcell * wz * l.x * wx * l.x;  // 1
      Lzxyy = kn_Vcell * nz * l.x * ny * l.y + kt_Vcell * tz * l.x * ty * l.y + kt_Vcell * wz * l.x * wy * l.y;  // 2
      Lzxzz = kn_Vcell * nz * l.x * nz * l.z + kt_Vcell * tz * l.x * tz * l.z + kt_Vcell * wz * l.x * wz * l.z;  // 3
      Lzxxy = kn_Vcell * nz * l.x * nx * l.y + kt_Vcell * tz * l.x * tx * l.y + kt_Vcell * wz * l.x * wx * l.y;  // 4
      Lzxxz = kn_Vcell * nz * l.x * nx * l.z + kt_Vcell * tz * l.x * tx * l.z + kt_Vcell * wz * l.x * wx * l.z;  // 5
      Lzxyz = kn_Vcell * nz * l.x * ny * l.z + kt_Vcell * tz * l.x * ty * l.z + kt_Vcell * wz * l.x * wy * l.z;  // 6
      Lzxyx = kn_Vcell * nz * l.x * ny * l.x + kt_Vcell * tz * l.x * ty * l.x + kt_Vcell * wz * l.x * wy * l.x;  // 7
      Lzxzx = kn_Vcell * nz * l.x * nz * l.x + kt_Vcell * tz * l.x * tz * l.x + kt_Vcell * wz * l.x * wz * l.x;  // 8
      Lzxzy = kn_Vcell * nz * l.x * nz * l.y + kt_Vcell * tz * l.x * tz * l.y + kt_Vcell * wz * l.x * wz * l.y;  // 9

      // zy
      Lzyxx = kn_Vcell * nz * l.y * nx * l.x + kt_Vcell * tz * l.y * tx * l.x + kt_Vcell * wz * l.y * wx * l.x;  // 1
      Lzyyy = kn_Vcell * nz * l.y * ny * l.y + kt_Vcell * tz * l.y * ty * l.y + kt_Vcell * wz * l.y * wy * l.y;  // 2
      Lzyzz = kn_Vcell * nz * l.y * nz * l.z + kt_Vcell * tz * l.y * tz * l.z + kt_Vcell * wz * l.y * wz * l.z;  // 3
      Lzyxy = kn_Vcell * nz * l.y * nx * l.y + kt_Vcell * tz * l.y * tx * l.y + kt_Vcell * wz * l.y * wx * l.y;  // 4
      Lzyxz = kn_Vcell * nz * l.y * nx * l.z + kt_Vcell * tz * l.y * tx * l.z + kt_Vcell * wz * l.y * wx * l.z;  // 5
      Lzyyz = kn_Vcell * nz * l.y * ny * l.z + kt_Vcell * tz * l.y * ty * l.z + kt_Vcell * wz * l.y * wy * l.z;  // 6
      Lzyyx = kn_Vcell * nz * l.y * ny * l.x + kt_Vcell * tz * l.y * ty * l.x + kt_Vcell * wz * l.y * wy * l.x;  // 7
      Lzyzx = kn_Vcell * nz * l.y * nz * l.x + kt_Vcell * tz * l.y * tz * l.x + kt_Vcell * wz * l.y * wz * l.x;  // 8
      Lzyzy = kn_Vcell * nz * l.y * nz * l.y + kt_Vcell * tz * l.y * tz * l.y + kt_Vcell * wz * l.y * wz * l.y;  // 9
    }

    // xx
    L[xx][xx] += Lxxxx;
    Lkn1 += Lknxxxx;
    Lkt1 += Lktxxxx;
    L[xx][yy] += Lxxyy;
    Lkn2 += Lknxxyy;
    Lkt2 += Lktxxyy;
    L[xx][zz] += Lxxzz;
    L[xx][xy] += Lxxxy;
    L[xx][xz] += Lxxxz;
    L[xx][yz] += Lxxyz;
    L[xx][yx] += Lxxyx, L[xx][zx] += Lxxzx;
    L[xx][zy] += Lxxzy;

    // yy
    L[yy][xx] += Lyyxx;
    L[yy][yy] += Lyyyy;
    L[yy][zz] += Lyyzz;
    L[yy][xy] += Lyyxy;
    L[yy][xz] += Lyyxz;
    L[yy][yz] += Lyyyz;
    L[yy][yx] += Lyyyx, L[yy][zx] += Lyyzx;
    L[yy][zy] += Lyyzy;

    // zz
    L[zz][xx] += Lzzxx;
    L[zz][yy] += Lzzyy;
    L[zz][zz] += Lzzzz;
    L[zz][xy] += Lzzxy;
    L[zz][xz] += Lzzxz;
    L[zz][yz] += Lzzyz;
    L[zz][yx] += Lzzyx, L[zz][zx] += Lzzzx;
    L[zz][zy] += Lzzzy;

    // xy
    L[xy][xx] += Lxyxx;
    L[xy][yy] += Lxyyy;
    L[xy][zz] += Lxyzz;
    L[xy][xy] += Lxyxy;
    L[xy][xz] += Lxyxz;
    L[xy][yz] += Lxyyz;
    L[xy][yx] += Lxyyx, L[xy][zx] += Lxyzx;
    L[xy][zy] += Lxyzy;

    // xz
    L[xz][xx] += Lxzxx;
    L[xz][yy] += Lxzyy;
    L[xz][zz] += Lxzzz;
    L[xz][xy] += Lxzxy;
    L[xz][xz] += Lxzxz;
    L[xz][yz] += Lxzyz;
    L[xz][yx] += Lxzyx, L[xz][zx] += Lxzzx;
    L[xz][zy] += Lxzzy;

    // yz
    L[yz][xx] += Lyzxx;
    L[yz][yy] += Lyzyy;
    L[yz][zz] += Lyzzz;
    L[yz][xy] += Lyzxy;
    L[yz][xz] += Lyzxz;
    L[yz][yz] += Lyzyz;
    L[yz][yx] += Lyzyx, L[yz][zx] += Lyzzx;
    L[yz][zy] += Lyzzy;

    // yx
    L[yx][xx] += Lyxxx;
    L[yx][yy] += Lyxyy;
    L[yx][zz] += Lyxzz;
    L[yx][xy] += Lyxxy;
    L[yx][xz] += Lyxxz;
    L[yx][yz] += Lyxyz;
    L[yx][yx] += Lyxyx, L[yx][zx] += Lyxzx;
    L[yx][zy] += Lyxzy;

    // zx
    L[zx][xx] += Lzxxx;
    L[zx][yy] += Lzxyy;
    L[zx][zz] += Lzxzz;
    L[zx][xy] += Lzxxy;
    L[zx][xz] += Lzxxz;
    L[zx][yz] += Lzxyz;
    L[zx][yx] += Lzxyx, L[zx][zx] += Lzxzx;
    L[zx][zy] += Lzxzy;

    // zy
    L[zy][xx] += Lzyxx;
    L[zy][yy] += Lzyyy;
    L[zy][zz] += Lzyzz;
    L[zy][xy] += Lzyxy;
    L[zy][xz] += Lzyxz;
    L[zy][yz] += Lzyyz;
    L[zy][yx] += Lzyyx, L[zy][zx] += Lzyzx;
    L[zy][zy] += Lzyzy;
  }
}

// Be carefull!! The density needs to be set before calling this function
void PBC3Dbox::initLagamine(double Q[]) {
  enableSwitch = 0;

  // Q[0] (in plane strain condition) unused here because the computation is 3D
  size_t offset = 1;
  size_t npa = (size_t)Q[offset++];

  // Particles
  Particles.clear();
  Particle P;
  for (size_t i = 0; i < npa; i++) {
    P.pos.x = Q[offset++];
    P.pos.y = Q[offset++];
    P.pos.z = Q[offset++];
    P.Q.s = Q[offset++];
    P.Q.v.x = Q[offset++];
    P.Q.v.y = Q[offset++];
    P.Q.v.z = Q[offset++];
    P.radius = Q[offset++];
    P.mass = (4.0 / 3.0) * M_PI * P.radius * P.radius * P.radius * density;  // Remember 'density' has to be set
    // before calling this function
    P.inertia = (2.0 / 5.0) * P.mass * P.radius * P.radius;
    Particles.push_back(P);
  }

  // Cell dimension
  Cell.h.xx = Q[offset++];
  Cell.h.xy = Q[offset++];
  Cell.h.xz = Q[offset++];
  Cell.h.yx = Q[offset++];
  Cell.h.yy = Q[offset++];
  Cell.h.yz = Q[offset++];
  Cell.h.zx = Q[offset++];
  Cell.h.zy = Q[offset++];
  Cell.h.zz = Q[offset++];
  Cell.mass = 1.0;  // Actually not used because the velocity (vh) is imposed

  // Contact list
  nbActiveInteractions = (size_t)Q[offset++];
  Interactions.clear();
  Interaction I;
  for (size_t i = 0; i < nbActiveInteractions; i++) {
    I.i = static_cast<size_t>(fabs(Q[offset++] - 1));  // Remenber that Fortran first index is 1
    I.j = static_cast<size_t>(fabs(Q[offset++] - 1));
    I.gap0 = Q[offset++];
    I.n.x = Q[offset++];
    I.n.y = Q[offset++];
    I.n.z = Q[offset++];
    I.fn = Q[offset++];
    I.fn_elas = Q[offset++];
    I.fn_bond = Q[offset++];
    I.ft.x = Q[offset++];
    I.ft.y = Q[offset++];
    I.ft.z = Q[offset++];
    I.ft_fric.x = Q[offset++];
    I.ft_fric.y = Q[offset++];
    I.ft_fric.z = Q[offset++];
    I.ft_bond.x = Q[offset++];
    I.ft_bond.y = Q[offset++];
    I.ft_bond.z = Q[offset++];
    I.mom.x = Q[offset++];
    I.mom.y = Q[offset++];
    I.mom.z = Q[offset++];
    I.state = static_cast<int>(Q[offset++]);
    I.D = Q[offset++];

    // std::cout << "STATE" << I.state << std::endl;
    Interactions.push_back(I);
  }
  nbBondsini = static_cast<int>(Q[offset++]);
  porosityini = Q[offset++];

  computeSampleData();
  dVerlet = 0.9 * Rmin;
}

// This function will run a simulation according to the current data in memory.
// For its use with Lagamine (multi-scale modeling) there's no logs to the screen.
void PBC3Dbox::runSilently() {
  // (re)-compute some constants in case they were not yet set
  dt_2 = 0.5 * dt;
  dt2_2 = 0.5 * dt * dt;

  updateNeighborList(dVerlet);

  while (t < tmax) {
    velocityVerletStep();

    if (interVerletC >= interVerlet) {
      updateNeighborList(dVerlet);
      interVerletC = 0.0;
    }

    interVerletC += dt;
    t += dt;
  }
}

// This version is for Lagamine (fortran)
void PBC3Dbox::transform(double dFmoinsI[3][3], double* /*I*/, int* nstep, int* iana, double* pressure,
                         double* /*sigRate*/) {
  // const double dtcFactor = 0.04; // 0.04 = 1/25

  double dtc = sqrt(Vmin * density / kn);
  dt = dtc * 0.1;  // dtc * dtcFactor;
  dt_2 = 0.5 * dt;
  dt2_2 = 0.5 * dt * dt;

  double max_dFmoinsI = fabs(dFmoinsI[0][0]);
  for (size_t i = 0; i < 3; i++) {
    for (size_t j = 0; j < 3; j++) {
      double tmp = fabs(dFmoinsI[i][j]);
      if (tmp > max_dFmoinsI) {
        max_dFmoinsI = tmp;
      }
    }
  }

  // int nstep = 10; //max_dFmoinsI * sqrt(Vmean / 1.0 / Rmean) / (*I) / dt;

  tmax = *nstep * dt;
  interVerlet = 1000 * dt;  // TODO: compute it according to a criterion

  double fact = 1.0 / (dt * (*nstep));

  mat9r vh;
  vh.xx = fact * (Cell.h.xx * dFmoinsI[0][0] + Cell.h.xy * dFmoinsI[1][0] + Cell.h.xz * dFmoinsI[2][0]);
  vh.xy = fact * (Cell.h.xx * dFmoinsI[0][1] + Cell.h.xy * dFmoinsI[1][1] + Cell.h.xz * dFmoinsI[2][1]);
  vh.xz = fact * (Cell.h.xx * dFmoinsI[0][2] + Cell.h.xy * dFmoinsI[1][2] + Cell.h.xz * dFmoinsI[2][2]);

  vh.yx = fact * (Cell.h.yx * dFmoinsI[0][0] + Cell.h.yy * dFmoinsI[1][0] + Cell.h.yz * dFmoinsI[2][0]);
  vh.yy = fact * (Cell.h.yx * dFmoinsI[0][1] + Cell.h.yy * dFmoinsI[1][1] + Cell.h.yz * dFmoinsI[2][1]);
  vh.yz = fact * (Cell.h.yx * dFmoinsI[0][2] + Cell.h.yy * dFmoinsI[1][2] + Cell.h.yz * dFmoinsI[2][2]);

  vh.zx = fact * (Cell.h.zx * dFmoinsI[0][0] + Cell.h.zy * dFmoinsI[1][0] + Cell.h.zz * dFmoinsI[2][0]);
  vh.zy = fact * (Cell.h.zx * dFmoinsI[0][1] + Cell.h.zy * dFmoinsI[1][1] + Cell.h.zz * dFmoinsI[2][1]);
  vh.zz = fact * (Cell.h.zx * dFmoinsI[0][2] + Cell.h.zy * dFmoinsI[1][2] + Cell.h.zz * dFmoinsI[2][2]);

  if (*iana == 2) {
    Load.VelocityControl(vh);
  } else if (*iana == 1) {
    Load.VelocityControlPlaneStress(vh, *pressure);
  }

  updateNeighborList(dVerlet);

  while (t < tmax) {
    velocityVerletStep();

    if (interVerletC >= interVerlet) {
      updateNeighborList(dVerlet);
      interVerletC = 0.0;
    }

    interVerletC += dt;
    t += dt;
  }
}

/// @brief hold a constant strain.
/// The strain is held while all components of the stress does not vary within
/// a given tolerance (tol) during a number of consecutive times (nstepConv).
/// If the number of steps reaches nstepMax, the computation is stopped anyway.
void PBC3Dbox::hold(double* tol, int* nstepConv, int* nstepMax, int* iana, double* pressure, double* /*sigRate*/) {
  mat9r previousSig = Sig;

  mat9r vh;
  vh.xx = 0;
  vh.xy = 0;
  vh.xz = 0;
  vh.yx = 0;
  vh.yy = 0;
  vh.yz = 0;
  vh.zx = 0;
  vh.zy = 0;
  vh.zz = 0;

  if (*iana == 2) {
    Load.VelocityControl(vh);
  } else if (*iana == 1) {
    Load.VelocityControlPlaneStress(vh, *pressure);
  }

  // std::cout << "tolSigma " << *tol << std::endl;
  // std::cout << "nstepconv " << *nstepConv << std::endl;
  // std::cout << "nstepmax " << *nstepMax << std::endl;
  updateNeighborList(dVerlet);

  int nstep = 0;
  int nstepOK = 0;
  while (nstep < *nstepMax) {
    velocityVerletStep();

    if ((Sig.xx - previousSig.xx) < *tol  // normalized by the confinement pressure
        && (Sig.xy - previousSig.xy) < *tol && (Sig.xz - previousSig.xz) < *tol && (Sig.yy - previousSig.yy) < *tol &&
        (Sig.yz - previousSig.yz) < *tol && (Sig.zz - previousSig.zz) < *tol) {
      nstepOK++;
    } else {
      nstepOK = 0;
    }  // nstepOK has to be consecutive!

    if (nstepOK >= *nstepConv) {
      break;
    }

    if (nstepOK >= *nstepConv) {
      break;
    }
    if (nstep == *nstepMax - 1) {
      std::cout << "******************************" << std::endl;
      std::cout << "nstepOK " << nstepOK << std::endl;
      std::cout << "error " << Sig.xx - previousSig.xx << std::endl;
      std::cout << "nstep " << nstep << std::endl;
      std::cout << "tol " << *tol << std::endl;
    }

    if (interVerletC >= interVerlet) {
      updateNeighborList(dVerlet);
      interVerletC = 0.0;
    }

    previousSig = Sig;
    interVerletC += dt;
    t += dt;
    nstep++;
  }
  if (nstep > 2000) {
    std::cout << "large nstep " << nstep << std::endl;
  }
  /*
  std::cout << "nstep " << nstep << std::endl;
  std::cout << "nstepconv " << *nstepConv << std::endl;
  std::cout << "nstepOK " << nstepOK << std::endl;
  std::cout << "nstepMax " << *nstepMax << std::endl;
  std::cout << "error " << (Sig.xx - previousSig.xx) << std::endl;
  std::cout << "error " << (Sig.xy - previousSig.xy) << std::endl;
  std::cout << "error " << (Sig.xz - previousSig.xz) << std::endl;
  std::cout << "error " << (Sig.yy - previousSig.yy) << std::endl;
  std::cout << "error " << (Sig.yz - previousSig.yz) << std::endl;
  std::cout << "error " << (Sig.zz - previousSig.zz) << std::endl;
  */
}

/// @brief A function that call the function 'transform' and then the function 'hold'
void PBC3Dbox::transform_and_hold(double dFmoinsI[3][3], double* I, double* tol, int* nstepConv, int* nstepMax,
                                  int* nstep, int* iana, double* pressure, double* sigRate) {
  transform(dFmoinsI, I, nstep, iana, pressure, sigRate);
  hold(tol, nstepConv, nstepMax, iana, pressure, sigRate);
}

/// @brief get the data back from PBC3D to Lagamine
void PBC3Dbox::endLagamine(double Q[], double SIG[3][3]) {
  size_t offset = 0;
  Q[offset++] = 0;  // 0 for 3D (this was used in 2D to say wether the in plane strain condition was used)

  // Particles
  Q[offset++] = (double)Particles.size();
  for (size_t i = 0; i < Particles.size(); i++) {
    Q[offset++] = Particles[i].pos.x;
    Q[offset++] = Particles[i].pos.y;
    Q[offset++] = Particles[i].pos.z;
    Q[offset++] = Particles[i].Q.s;
    Q[offset++] = Particles[i].Q.v.x;
    Q[offset++] = Particles[i].Q.v.y;
    Q[offset++] = Particles[i].Q.v.z;
    Q[offset++] = Particles[i].radius;
  }

  // Cell dimension (components)
  Q[offset++] = Cell.h.xx;
  Q[offset++] = Cell.h.xy;
  Q[offset++] = Cell.h.xz;
  Q[offset++] = Cell.h.yx;
  Q[offset++] = Cell.h.yy;
  Q[offset++] = Cell.h.yz;
  Q[offset++] = Cell.h.zx;
  Q[offset++] = Cell.h.zy;
  Q[offset++] = Cell.h.zz;

  // number of contact is updated here since its incremental calculation in computeForcesAndMoments() is sometimes
  // missing a few contacts (???)
  nbActiveInteractions = 0;
  nbBonds = 0;
  for (size_t k = 0; k < Interactions.size(); k++) {
    if (Interactions[k].state == noContactState) {
      continue;
    }
    if (Interactions[k].state == bondedState) {
      nbBonds++;
    }
    if (Interactions[k].state == bondedStateDam) {
      nbBonds++;
    }
    nbActiveInteractions++;
  }
  Q[offset++] = (double)nbActiveInteractions;
  for (size_t k = 0; k < Interactions.size(); k++) {
    if (Interactions[k].state == noContactState) {
      continue;
    }
    Q[offset++] = (double)(Interactions[k].i + 1);  // Fortran index starts with 1
    Q[offset++] = (double)(Interactions[k].j + 1);
    Q[offset++] = Interactions[k].gap0;

    Q[offset++] = Interactions[k].n.x;
    Q[offset++] = Interactions[k].n.y;
    Q[offset++] = Interactions[k].n.z;
    Q[offset++] = Interactions[k].fn;
    Q[offset++] = Interactions[k].fn_elas;
    Q[offset++] = Interactions[k].fn_bond;
    Q[offset++] = Interactions[k].ft.x;
    Q[offset++] = Interactions[k].ft.y;
    Q[offset++] = Interactions[k].ft.z;
    Q[offset++] = Interactions[k].ft_fric.x;
    Q[offset++] = Interactions[k].ft_fric.y;
    Q[offset++] = Interactions[k].ft_fric.z;
    Q[offset++] = Interactions[k].ft_bond.x;
    Q[offset++] = Interactions[k].ft_bond.y;
    Q[offset++] = Interactions[k].ft_bond.z;
    Q[offset++] = Interactions[k].mom.x;
    Q[offset++] = Interactions[k].mom.y;
    Q[offset++] = Interactions[k].mom.z;
    Q[offset++] = Interactions[k].state;
    Q[offset++] = Interactions[k].D;
  }
  // state variables: this is added to send back some state variables to Lagamine
  double Vcell = fabs(Cell.h.det());
  // update initial values if equal to -1 (specified in the REVini file, ugly but works...)
  if (nbBondsini == -1) {
    nbBondsini = nbBonds;
  }
  Q[offset++] = (double)nbBondsini;
  if (porosityini == -1) {
    porosityini = (Vcell - Vsolid) / Vcell;
  }
  Q[offset++] = porosityini;

  Q[offset++] = (double)nbBonds;
  Q[offset++] = (Vcell - Vsolid) / Vcell;
  Q[offset++] =
      (double)nbActiveInteractions / (double)Particles.size();  // to plot the evolution of coordination number
  double ResMean, Res0Mean, fnMin, fnMean;
  staticQualityData(&ResMean, &Res0Mean, &fnMin, &fnMean);

  // std::cout << "fnmin*** " << fnMin << std::endl;
  Q[offset++] = fnMin;
  // end state variables

  // Stress
  SIG[0][0] = Sig.xx;
  SIG[0][1] = Sig.xy;
  SIG[0][2] = Sig.xz;
  SIG[1][0] = Sig.yx;
  SIG[1][1] = Sig.yy;
  SIG[1][2] = Sig.yz;
  SIG[2][0] = Sig.zx;
  SIG[2][1] = Sig.zy;
  SIG[2][2] = Sig.zz;
}

/// @brief Computes some indicators related to the staticity
/// @param[out]  ResMean    Average resultant force over all bodies
/// @param[out]  Res0Mean   Same as ResMean but with only the body holding more than 2 contacts
/// @param[out]  fnMin    Minimum nonzero normal force
/// @param[out]  fnMean   Mean nonzero normal force
void PBC3Dbox::staticQualityData(double* ResMean, double* Res0Mean, double* fnMin, double* fnMean) const {
  *ResMean = *Res0Mean = *fnMin = *fnMean = 0.0;

  size_t Nc = Interactions.size();
  size_t Np = Particles.size();

  if (Nc == 0 || Np == 0) {
    return;
  }

  std::vector<vec3r> R(Particles.size());
  for (size_t i = 0; i < R.size(); i++) {
    R[i].reset();
  }

  std::vector<int> z(Np, 0);
  for (size_t k = 0; k < Interactions.size(); k++) {
    if (Interactions[k].state == noContactState) {
      continue;
    }
    z[Interactions[k].i] += 1;
    z[Interactions[k].j] += 1;
  }

  *fnMin = 1e12;
  size_t Nc0 = 0;
  for (size_t k = 0; k < Interactions.size(); k++) {
    if (Interactions[k].state == noContactState) {
      continue;
    }
    if (z[Interactions[k].i] < 3 || z[Interactions[k].j] < 3) {
      continue;
    }

    Nc0++;
    *fnMean += Interactions[k].fn;
    if (Interactions[k].fn > 0.0 && Interactions[k].fn < *fnMin) {
      *fnMin = Interactions[k].fn;
    }

    vec3r f = Interactions[k].fn * Interactions[k].n + Interactions[k].ft;
    R[Interactions[k].i] += f;
    R[Interactions[k].j] -= f;
  }

  if (Nc0 > 0) {
    *fnMean /= (double)Nc0;
  }

  size_t Np0 = 0;
  for (size_t i = 0; i < R.size(); i++) {
    double len = R[i].length();
    *ResMean += len;
    if (z[i] > 2) {
      Np0++;
      *Res0Mean += len;
    }
  }
  *ResMean /= (double)Np;
  if (Np0 > 0) {
    *Res0Mean /= (double)Np0;
  }
}
