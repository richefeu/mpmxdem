// =================================================
// Periodic boundary conditions with clumped spheres
// =================================================

#include "PBC3D.hpp"

PBC3Dbox::PBC3Dbox() {
  // Some default values (actually, most of them will be (re-)set after)
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
  kn = 1e4;
  kt = kn;
  kr = 0.0;
  dampRate = 0.95;
  mu = 0.8;
  mur = 0.0;
  fcoh = 0.0;
  iconf = 0;
  nbActiveInteractions = 0;
  nbBonds = 0;
  numericalDampingCoeff = 0.0;
}

/// @brief Print a banner related with the current code
void PBC3Dbox::showBanner() {
  std::cout << std::endl;
  std::cout << "PBC3Dbox_snow, Periodic Boundary Conditions in 3D with cohesive clumps for snow microstructure"
            << std::endl;
  std::cout << "<Vincent.Richefeu@3sr-grenoble.fr>" << std::endl;
  std::cout << std::endl;
}

/// @brief Clear Particles and Interactions
void PBC3Dbox::clearMemory() {
  Particles.clear();
  Interactions.clear();
}

/// @brief Save the current configuration
/// @param[in] i File number. It will be named 'confx' where x is replaced by i
void PBC3Dbox::saveConf(int i) {
  char fname[256];
  sprintf(fname, "conf%d", i);
  std::ofstream conf(fname);

  conf << "PBC3D 30-01-2019\n";  // format: progName version-date
  conf << "t " << t << '\n';
  conf << "tmax " << tmax << '\n';
  conf << "dt " << dt << '\n';
  conf << "interVerlet " << interVerlet << '\n';
  conf << "interOut " << interOut << '\n';
  conf << "interConf " << interConf << '\n';
  conf << "dVerlet " << dVerlet << '\n';
  conf << "density " << density << '\n';
  conf << "kn " << kn << '\n';
  conf << "kt " << kt << '\n';
  conf << "kr " << kr << '\n';
  conf << "dampRate " << dampRate << '\n';
  conf << "mu " << mu << '\n';
  conf << "mur " << mur << '\n';
  conf << "fcoh " << fcoh << '\n';
  conf << "iconf " << iconf << '\n';
  conf << "h " << Cell.h << '\n';
  conf << "vh " << Cell.vh << '\n';
  conf << "ah " << Cell.ah << '\n';
  conf << "hmass " << Cell.mass << '\n';
  conf << "enableSwitch " << enableSwitch << '\n';
  if (numericalDampingCoeff != 0) conf << "numericalDampingCoeff " << numericalDampingCoeff << '\n';
  conf << "Load " << Load.StoredCommand << '\n';
  conf << "Particles " << Particles.size() << '\n';
  for (size_t i = 0; i < Particles.size(); i++) {
    conf << Particles[i].pos << ' ' << Particles[i].vel << ' ' << Particles[i].acc << ' ' << Particles[i].Q << ' '
         << Particles[i].vrot << ' ' << Particles[i].arot << '\n';
  }
  conf << "Interactions " << nbActiveInteractions << '\n';
  for (size_t i = 0; i < Interactions.size(); i++) {
    if (Interactions[i].state == noContactState) continue;
    conf << Interactions[i].i << ' ' << Interactions[i].j << ' ' << Interactions[i].is << ' ' << Interactions[i].js
         << ' ' << Interactions[i].gap0 << ' ' << Interactions[i].n << ' ' << Interactions[i].fn << ' '
         << Interactions[i].fn_elas << ' ' << Interactions[i].fn_bond << ' ' << Interactions[i].ft << ' '
         << Interactions[i].ft_fric << ' ' << Interactions[i].ft_bond << ' ' << Interactions[i].dt_fric << ' '
         << Interactions[i].dt_bond << ' ' << Interactions[i].drot_bond << ' ' << Interactions[i].mom << ' '
         << Interactions[i].dampn << ' ' << Interactions[i].dampt << ' ' << Interactions[i].state << ' '
         << Interactions[i].D << '\n';
  }
}

/// @brief Load the configuration
/// @param[in]    name     Name of the file
void PBC3Dbox::loadConf(const char* name) {
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
  if (date != "30-01-2019") {
    std::cerr << "@PBC3Dbox, The version-date should be 18-04-2018!" << std::endl;
  }

  std::string token;
  conf >> token;
  while (conf.good()) {
    if (token[0] == '/' || token[0] == '#' || token[0] == '!') {
      getline(conf, token);  // ignore the rest of the current line
      conf >> token;         // next token
      continue;
    } else if (token == "t")
      conf >> t;
    else if (token == "tmax")
      conf >> tmax;
    else if (token == "dt") {
      conf >> dt;
      dt_2 = 0.5 * dt;
      dt2_2 = 0.5 * dt * dt;
    } else if (token == "interVerlet")
      conf >> interVerlet;
    else if (token == "interOut")
      conf >> interOut;
    else if (token == "interConf")
      conf >> interConf;
    else if (token == "dVerlet")
      conf >> dVerlet;
    else if (token == "density")
      conf >> density;
    else if (token == "kn")
      conf >> kn;
    else if (token == "kt")
      conf >> kt;
    else if (token == "kr")
      conf >> kr;
    else if (token == "dampRate")
      conf >> dampRate;
    else if (token == "mu")
      conf >> mu;
    else if (token == "mur")
      conf >> mur;
    else if (token == "fcoh")
      conf >> fcoh;
    else if (token == "iconf")
      conf >> iconf;
    else if (token == "h")
      conf >> Cell.h;
    else if (token == "vh")
      conf >> Cell.vh;
    else if (token == "ah")
      conf >> Cell.ah;
    else if (token == "hmass")
      conf >> Cell.mass;
    else if (token == "enableSwitch")
      conf >> enableSwitch;
    else if (token == "numericalDampingCoeff")
      conf >> numericalDampingCoeff;
    else if (token == "Load") {
      std::string command;
      conf >> command;
      if (command == "TriaxialCompressionY") {
        double pressure, velocity;
        conf >> pressure >> velocity;
        Load.TriaxialCompressionY(pressure, velocity);
      } else if (command == "TriaxialCompressionZ") {
        double pressure, velocity;
        conf >> pressure >> velocity;
        Load.TriaxialCompressionZ(pressure, velocity);
      } else if (command == "BiaxialCompressionYPlaneStrainZ") {
        double pressure, velocity;
        conf >> pressure >> velocity;
        Load.BiaxialCompressionYPlaneStrainZ(pressure, velocity);
      } else if (command == "BiaxialCompressionZPlaneStrainX") {
        double pressure, velocity;
        conf >> pressure >> velocity;
        Load.BiaxialCompressionZPlaneStrainX(pressure, velocity);
      } else if (command == "IsostaticCompression") {
        double pressure;
        conf >> pressure;
        Load.IsostaticCompression(pressure);
      } else if (command == "VelocityControl") {
        mat9r vh;
        conf >> vh;
        Load.VelocityControl(vh);
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
      } else {
        std::cerr << "Unknown command for loading: " << command << std::endl;
      }
    } else if (token == "Particles") {
      size_t nb;
      conf >> nb;
      Particles.clear();
      Particle P;
      for (size_t i = 0; i < nb; i++) {
        // remember that pos, vel and acc are expressed in reduced coordinates
        conf >> P.pos >> P.vel >> P.acc >> P.Q >> P.vrot >> P.arot;
        Particles.push_back(P);
      }
    } else if (token == "Interactions") {
      size_t nb;
      conf >> nb;
      Interactions.clear();
      Interaction I;
      for (size_t i = 0; i < nb; i++) {
        conf >> I.i >> I.j >> I.is >> I.js >> I.gap0 >> I.n >> I.fn >> I.fn_elas >> I.fn_bond >> I.ft >> I.ft_fric >>
            I.ft_bond >> I.dt_fric >> I.dt_bond >> I.drot_bond >> I.mom >> I.dampn >> I.dampt >> I.state >> I.D;
        Interactions.push_back(I);
      }
    }
    /// ============= PRE-PROCESSING ==============
    else if (token == "RandomVelocities") {
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

  loadShapes();

  // computeSampleData();
  accelerations();  // a fake time-increment that will compute missing thinks
}

void PBC3Dbox::loadShapes() {
  std::ifstream shp("shapes.txt");
  if (!shp.is_open()) {
    std::cerr << "@PBC3Dbox::loadShapes, Cannot read shapes.txt";
  }

  size_t current = 0;

  std::string token;
  shp >> token;
  while (shp.good()) {
    if (token[0] == '/' || token[0] == '#' || token[0] == '!') {
      getline(shp, token);  // ignore the rest of the current line
      shp >> token;         // next token
      continue;
    } else if (token == "<") {
      if (current < Particles.size()) {
        Particles[current].readShape(shp, density);
        current++;
      } else {
        std::cout << "More shapes that clumps!\n";
      }
    } else {
      std::cerr << "Unknown token: " << token << std::endl;
      exit(0);
    }

    shp >> token;
  }

  if (current != Particles.size()) {
    std::cout << "The number of particles does not egal the number of shapes!\n";
  }
}

/// @brief Computes a single step with the velocity-Verlet algorithm
void PBC3Dbox::velocityVerletStep() {
  if (Load.ServoFunction != nullptr) Load.ServoFunction(*this);

  for (size_t i = 0; i < Particles.size(); i++) {
    Particles[i].pos += dt * Particles[i].vel + dt2_2 * Particles[i].acc;
    Particles[i].vel += dt_2 * Particles[i].acc;

    // Periodicity in position (can be usefull in the sample preparation)
    if (enableSwitch > 0) {
      for (size_t c = 0; c < 3; c++) {
        while (Particles[i].pos[c] < 0.0) Particles[i].pos[c] += 1.0;
        while (Particles[i].pos[c] > 1.0) Particles[i].pos[c] -= 1.0;
      }
      // Remark: the reduced velocities do not need to be corrected
      //         since they are periodic
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
  vmean /= (double)(Particles.size());
  for (size_t i = 0; i < Particles.size(); i++) {
    Particles[i].vel -= vmean;
  }

  for (size_t c = 0; c < 9; c++) {
    if (Load.Drive[c] == ForceDriven) Cell.vh[c] += dt_2 * Cell.ah[c];
  }

  Cell.update(dt);
}

/// @brief Print information about the running computation and current state of the sample
void PBC3Dbox::printScreen(double elapsedTime) {
  std::cout << "+===============================================================================" << '\n';
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

  /*
        double Rmean, R0mean, fnMin, fnMean;
  staticQualityData(&Rmean, &R0mean, &fnMin, &fnMean);
  std::cout << "|  Mean resultant: " << Rmean << ", R0mean: " << R0mean << ", fnMin:  " << fnMin
            << ", fnMean:  " << fnMean << '\n';
  */

  std::cout << "+===============================================================================" << '\n';
  std::cout << '\n' << std::endl;
}

/// @brief The main loop of time-integration
void PBC3Dbox::integrate() {
  // (re)-compute some constants in case they were not yet set
  dt_2 = 0.5 * dt;
  dt2_2 = 0.5 * dt * dt;
  accelerations();
  // dataOutput();

  char fname[256];
  sprintf(fname, "conf%d", iconf);
  if (!fileTool::fileExists(fname)) {
    saveConf(iconf);
  }

  double previousTime = (double)std::clock() / (double)CLOCKS_PER_SEC;
  while (t < tmax) {
    velocityVerletStep();

    if (interConfC >= interConf) {
      iconf++;

      double currentTime = (double)std::clock() / (double)CLOCKS_PER_SEC;
      printScreen(currentTime - previousTime);
      previousTime = currentTime;

      saveConf(iconf);
      interConfC = 0.0;
    }

    if (interVerletC >= interVerlet) {
      updateNeighborList(dVerlet);
      interVerletC = 0.0;
    }

    interConfC += dt;
    interOutC += dt;
    interVerletC += dt;
    t += dt;
  }

  return;
}

void PBC3Dbox::getSubSpheres(vec3r& branch, size_t i, size_t j, std::vector<std::pair<size_t, size_t> >& duoIDs) {
  // here we could pre-select the spheres that are close with a fast algorithm
  // Now we use the brute-forte strategy

  duoIDs.clear();
  std::pair<size_t, size_t> duoID;
  for (size_t is = 0; is < Particles[i].subSpheres.size(); is++) {
    for (size_t js = 0; js < Particles[j].subSpheres.size(); js++) {
      duoID.first = is;
      duoID.second = js;
      duoIDs.push_back(duoID);
    }
  }
}

/// @brief  Update the neighbor list (that is the list of 'active' and 'non-active' interactions)
/// @param[in] dmax Maximum distance for adding an Interaction in the neighbor list
void PBC3Dbox::updateNeighborList(double dmax) {
  // std::cout << "Update NL\n";

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
      for (size_t c = 0; c < 3; c++) sij[c] -= floor(sij[c] + 0.5);
      vec3r branch = Cell.h * sij;

      // get list of posible subSpheres that can interact
      std::vector<std::pair<size_t, size_t> > duoIDs;
      getSubSpheres(branch, i, j, duoIDs);
      // std::cout << "duoIDs.size() = " << duoIDs.size() << "\n";

      double m = (Particles[i].mass * Particles[j].mass) / (Particles[i].mass + Particles[j].mass);
      double Dampn = dampRate * 2.0 * sqrt(kn * m);
      double Dampt = dampRate * 2.0 * sqrt(kt * m);
      for (size_t k = 0; k < duoIDs.size(); k++) {
        size_t is = duoIDs[k].first;
        size_t js = duoIDs[k].second;
        vec3r posi = Particles[i].Q * Particles[i].subSpheres[is].localPos;
        double Ri = Particles[i].subSpheres[is].radius;
        vec3r posj = branch + Particles[j].Q * Particles[j].subSpheres[js].localPos;
        double Rj = Particles[j].subSpheres[js].radius;
        vec3r sbranch = posj - posi;

        double sum = dmax + Ri + Rj;
        if (norm2(sbranch) <= sum * sum) {
          Interactions.push_back(Interaction(i, j, is, js, Dampn, Dampt));
        }
      }
    }
  }
  // std::cout << "Interactions.size() = " << Interactions.size() << "\n";

  // retrieve previous contacts or bonds
  size_t k, kold = 0;
  for (k = 0; k < Interactions.size(); ++k) {
    while (kold < Ibak.size() && Ibak[kold].i < Interactions[k].i) ++kold;
    if (kold == Ibak.size()) break;

    while (kold < Ibak.size() && Ibak[kold].i == Interactions[k].i && Ibak[kold].j < Interactions[k].j) ++kold;
    if (kold == Ibak.size()) break;

    while (kold < Ibak.size() && Ibak[kold].i == Interactions[k].i && Ibak[kold].j == Interactions[k].j &&
           Ibak[kold].is < Interactions[k].is)
      ++kold;
    if (kold == Ibak.size()) break;

    while (kold < Ibak.size() && Ibak[kold].i == Interactions[k].i && Ibak[kold].j == Interactions[k].j &&
           Ibak[kold].is == Interactions[k].is && Ibak[kold].js < Interactions[k].js)
      ++kold;
    if (kold == Ibak.size()) break;

    if (Ibak[kold].i == Interactions[k].i && Ibak[kold].j == Interactions[k].j && Ibak[kold].is == Interactions[k].is &&
        Ibak[kold].js == Interactions[k].js) {
      Interactions[k] = Ibak[kold];
      ++kold;
    }
  }
}

/// @brief Compute acceleration of the particles and of the periodic-cell.
void PBC3Dbox::accelerations() {
  // Set forces and moments to zero
  for (size_t i = 0; i < Particles.size(); i++) {
    Particles[i].force.reset();
    Particles[i].moment.reset();
  }
  Sig.reset();  // reset internal stress

  nbActiveInteractions = 0;
  nbBonds = 0;

  computeForcesAndMoments();

  // Cundall damping
  if (numericalDampingCoeff > 0.0) {
    for (size_t i = 0; i < Particles.size(); i++) {
      double factor;
      double factorMinus = 1.0 - numericalDampingCoeff;
      double factorPlus = 1.0 + numericalDampingCoeff;
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

  // Finally compute the accelerations (translation and rotation) of the particles
  for (size_t i = 0; i < Particles.size(); i++) {
    vec3r acc = Particles[i].force / Particles[i].mass;

#if 0
    // =====================================================
    // The following 2 lines can be removed.
    // In fact, mathematic relations say we should use them
    // but in practice their presence makes no difference 
    // in the case of quasistatic strainning (ah and vh <<< 1)
    // =====================================================
    acc -= Cell.ah * Particles[i].pos;
    acc -= 2.0 * Cell.vh * Particles[i].vel;
#endif

    Particles[i].acc = hinv * acc;

    quat Qinv = Particles[i].Q.get_conjugated();
    vec3r omega = Qinv * Particles[i].vrot;  // Express omega in the body framework
    vec3r M = Qinv * Particles[i].moment;    // Express torque in the body framework
    vec3r inertia = Particles[i].I_m * Particles[i].mass;
    vec3r domega((M[0] - (inertia[2] - inertia[1]) * omega[1] * omega[2]) / inertia[0],
                 (M[1] - (inertia[0] - inertia[2]) * omega[2] * omega[0]) / inertia[1],
                 (M[2] - (inertia[1] - inertia[0]) * omega[0] * omega[1]) / inertia[2]);
    Particles[i].arot = Particles[i].Q * domega;  // Express arot in the global framework
  }
}

/// @brief Computes the interaction forces and moments,
///        and the tensorial moment (= Vcell * stress matrix) of the cell
void PBC3Dbox::computeForcesAndMoments() {
  size_t i, j, is, js;
  for (size_t k = 0; k < Interactions.size(); k++) {
    i = Interactions[k].i;
    j = Interactions[k].j;
    is = Interactions[k].is;
    js = Interactions[k].js;

    vec3r sij = Particles[j].pos - Particles[i].pos;
    sij.x -= floor(sij.x + 0.5);
    sij.y -= floor(sij.y + 0.5);
    sij.z -= floor(sij.z + 0.5);
    vec3r branch = Cell.h * sij;

    vec3r posi = Particles[i].Q * Particles[i].subSpheres[is].localPos;
    double Ri = Particles[i].subSpheres[is].radius;
    vec3r posj = branch + Particles[j].Q * Particles[j].subSpheres[js].localPos;
    double Rj = Particles[j].subSpheres[js].radius;
    vec3r sbranch = posj - posi;

    // ===========================================================
    // ======= A NON-BONDED FRICTIONAL CONTACT INTERACTION =======
    // ===========================================================

    double sum = Ri + Rj;
    if (norm2(sbranch) <= sum * sum) {  // it means that subSpheres is and js are in contact
      nbActiveInteractions++;
      Interactions[k].state = contactState;

      // Current normal vector (the previous one has previously been stored in Interactions[k].n)
      vec3r n = sbranch;
      double len = n.normalize();

      // real relative velocities
      double dn = len - Ri - Rj;
      vec3r ai = posi + (Ri + 0.5 * dn) * n;
      vec3r aj = ai - branch;
      vec3r vel = Particles[j].vel - Particles[i].vel;
      vec3r realVel = Cell.h * vel + Cell.vh * sij;
      realVel += cross(ai, Particles[i].vrot) - cross(aj, Particles[j].vrot);

      // Normal force (elastic + viscuous damping)
      double vn = realVel * n;
      double fne = -kn * dn;
      double fnv = -Interactions[k].dampn * vn;
      Interactions[k].fn = fne + fnv;
      // if (Interactions[k].fn < 0.0) Interactions[k].fn = 0.0;  // Because viscuous damping can make fn negative
      Interactions[k].fn_elas = Interactions[k].fn;

      // Tangential force (friction)
      vec3r vt = realVel - (vn * n);
      vec3r deltat = vt * dt;
      Interactions[k].dt_fric += deltat;
      Interactions[k].ft -= kt * deltat;                 // no viscuous damping since friction can dissipate
      double threshold = fabs(mu * Interactions[k].fn);  // Suppose that fn is elastic and without cohesion
      double ft_square = Interactions[k].ft * Interactions[k].ft;
      if (ft_square > 0.0 && ft_square >= threshold * threshold)
        Interactions[k].ft = threshold * Interactions[k].ft * (1.0f / sqrt(ft_square));
      Interactions[k].ft_fric = Interactions[k].ft;

      // Cohesion
      Interactions[k].fn += fcoh;

      // Resultant forces
      vec3r f = Interactions[k].fn * n + Interactions[k].ft;
      Particles[i].force -= f;
      Particles[j].force += f;

      // Resultant moments
      Particles[i].moment += cross(ai, f);
      Particles[j].moment += cross(aj, -f);

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

      // if (permamentGluer == 1) {
      //   // switch to a cemented/bonded link
      //   Interactions[k].state = bondedState;
      //
      //   if (dn >= 0.0)
      //     Interactions[k].gap0 = dn;
      //   else
      //     Interactions[k].gap0 = 0.0;
      // }
    } else {
      Interactions[k].dt_fric.reset();
      Interactions[k].state = noContactState;
    }
    // ===========================================================

  }  // Loop over interactions
}
