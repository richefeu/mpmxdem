#ifndef PBC3D_SANDSTONE_HPP
#define PBC3D_SANDSTONE_HPP

#include <algorithm>
#include <cmath>
#include <cstdio>   // printf
#include <cstdlib>  // rand
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <utility>
#include <vector>

#include "Interaction.hpp"
#include "Loading.hpp"
#include "Particle.hpp"
#include "PeriodicCell.hpp"
#include "fileTool.hpp"
#include "geoPack3D.hpp"

// This is useful to make fortran-like outputs
#define __FORMATED(P, W, V) std::fixed << std::setprecision(P) << std::setw(W) << std::left << (V)

/// 3D Periodic boundary conditions with spheres
class PBC3Dbox {
 public:
  std::vector<Particle> Particles;        ///< The particles
  std::vector<Interaction> Interactions;  ///< The interactions (particle IDs, frames, force, etc.)
  ///< In fact, it's the so-called neighbor-list because some interactions
  ///< are not active
  Loading Load;                 ///< The Loading
  PeriodicCell Cell;            ///< The periodic cell
  mat9r Sig;                    ///< Internal stress
  size_t nbActiveInteractions;  ///< Number of active contacts, ie all interactions without the "noContactState"
  ///< It can be different from Interactions.size()!

  double nbBondsini;   ///< initial # of Bonds at start of Lagamine
  double porosityini;  ///< initial porosity at start of Lagamine
  double nbBonds;
  double tensfailure;
  double fricfailure;

  // Time parameters
  double t;     ///< Current Time
  double tmax;  ///< End time
  double dt;    ///< Time increment

  // Simulation flow
  double interVerlet;  ///< Time intervalle between each update of the neighbor-list
  double interOut;     ///< Time intervalle between data outputs
  double interConf;    ///< Time intervalle between the CONF files

  // Neighbor list
  double dVerlet;  ///< Distance of Verlet

  // Properties
  double density;   ///< Density of all particles
  double kn;        ///< Normal stiffness (for compression/tension, bonded or not)
  double kt;        ///< Tangential stiffness (bonded or not)
  double kr;        ///< Angular stiffness (only for bonded links)
  double dampRate;  ///< Viscous damping Rate -- in the range [0, 1[
  double mu;        ///< Coefficent of friction
  double mur;       ///< Coefficient of "angular-friction"
  double fcoh;      ///< Cohesion force (strictly negative)
  double zetaMax;   ///< Can be seen as "dn_rupture / dn_dammage_starts"
  double Kratio;    ///< Ratio of particle stiffness over bond stiffness

  // Solid cohesion
  double fn0;      ///< Maximum normal force
  double ft0;      ///< Maximum tangential force
  double mom0;     ///< Maximum Torque
  double dn0;      ///< Maximum normal displacement
  double dt0;      ///< Maximum tangential displacement
  double drot0;    ///< Maximum angular rotation
  double powSurf;  ///< Power used in the breakage surface

  // Other parameters
  int iconf;           ///< Current configuration ID
  int enableSwitch;    ///< If non-null, enable the switch of particles from one boundary to the opposite
  int permamentGluer;  ///< If 1, contacts are permanently transformed to glued-point
  double numericalDampingCoeff;

  // Ctor
  PBC3Dbox();
  //PBC3Dbox(const PBC3Dbox& box);

  // Methods
  void showBanner();               ///< Displays a banner about the code
  void initOutputFiles();          ///< Opens output files that hold processed data (stress, fabric, etc.)
  void setSample();                ///< Creates a sample by asking questions to the user
  void velocityVerletStep();       ///< Makes a time increment with the velocity-Verlet scheme
  void integrate();                ///< Simulation flow
                                   ///< (iteratively make a time increment and check for updates or saving)
  void accelerations();            ///< Computes accelerations (both for particles and the periodic-cell)
  void computeForcesAndMoments();  ///< Computes forces and moments (and cell-stress)
  double YieldFuncDam(double zeta, double Dn, double DtNorm, double DrotNorm);
  ///< Used for interaction of type 'bondedStateDam'

  void printScreen(double elapsedTime);  ///< Prints usefull data on screen during computation
  void dataOutput();                     ///< Outputs usefull data during computation
  void updateNeighborList(double dmax);  ///< Updates the neighbor-list
  void saveConf(const char* name);
  void saveConf(int i);             ///< Saves the current configuration in a file named confX, where X=i
  void loadConf(const char* name);  ///< Loads a configuration from a file
  void clearMemory();               ///< Clears the Particles and Interactions.
  void computeSampleData();         ///< Computes a number of usefull data (Rmin, Rmax, Vsolid, etc.)
  void ActivateBonds(double epsiDist,
                     int state);  ///< Replace contacts by cemented bonds when dn is lower than epsiDist
  void RemoveBonds(double percentRemove, int StrategyId);  ///< ....

  void transform(mat9r& Finc, double macro_dt);  ///< for MPMxDEM double-scale simulation

  // Methods specifically written for Lagamine (FEMxDEM coupling).
  // They are compatible with fortran.
  void initLagamine(double Q[]);  ///< Kind of serialization solution to get the initial configuration from Lagamine
  void initLagamineSandstone(double Q[]);  ///< Gets the initial configuration from Lagamine
  void transform(double dFmoinsI[3][3], double* I, int* nstep);
  void hold(double* tol, int* nstepConv, int* nstepMax);
  void transform_and_hold(double dFmoinsI[3][3], double* I, double* tol, int* nstepConv, int* nstepMax, int* nstep);
  // void runBlindly();  ///< Runs the simulation so that it ends with a static equilibrium state
  void runSilently();  ///< Runs the simulation silently (without outputs) from time t to tmax
  void endLagamine(double Q[], double SIG[3][3]);           ///< Kind of serialization to get the data back
  void endLagamineSandstone(double Q[], double SIG[3][3]);  ///< Kind of serialization to get the data back
  void getOperatorKruyt(double L[6][6]);                    ///< Gets the operator proposed by Kruyt
  void getOperatorKruyt2(double L[9][9]);                   ///< Gets the operator proposed by Kruyt (version Kien)
  void getOperatorKruyt3(double L[9][9]);                   ///< Gets the operator proposed by Kruyt
  ///< (version Kien avec sliding-contacts)

  void staticQualityData(double* Rmean, double* R0mean, double* fnMin,
                         double* fnMean) const;  ///< Methods to evaluate the quality of static state

 private:
  // Files
  std::ofstream stressOut;     ///< File to store stress
  std::ofstream cellOut;       ///< File to store cell data
  std::ofstream strainOut;     ///< File to store strain
  std::ofstream resultantOut;  ///< File to store resultant data

  // Counters for the simulation flow
  double interVerletC;  ///< A counter for reconstruction of the neighbor list
  double interOutC;     ///< A counter for writting in output files
  double interConfC;    ///< A counter for writting conf files

  // time-step constants
  double dt_2;   ///< Half the time-step
  double dt2_2;  ///< Half the squared time-step

  //
  double w_bond;
  double w_particle;

 public:
  // Sample
  double Vsolid;  ///< Total volume of the particles
  double Vmin;    ///< Minimum volume of the particles
  double Vmax;    ///< Maximum volume of the particles
  double Vmean;   ///< Mean volume of the particles

  // Radii
  double Rmin;   ///< Minimum radius of the particles
  double Rmax;   ///< Maximum radius of the particles
  double Rmean;  ///< Mean radius of the particles

  // Normal forces
  double FnMin;   ///< Minimum normal force in the system
  double FnMax;   ///< Maximum normal force in the system
  double FnMean;  ///< Mean normal force in the system

  // Particle velocities
  double VelMin;   ///< Minimum velocity magnitude of the particles
  double VelMax;   ///< Maximum velocity magnitude of the particles
  double VelMean;  ///< Mean velocity magnitude of the particles
};

#endif /* end of include guard: PBC3D_SANDSTONE_HPP */
