#ifndef PBC3D_SANDSTONE_HPP
#define PBC3D_SANDSTONE_HPP

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <random>
#include <utility>
#include <vector>

#include "Interaction.hpp"
#include "Loading.hpp"
#include "Particle.hpp"
#include "PeriodicCell.hpp"

// from toofus
#include "Mth.hpp"
#include "fileTool.hpp"
#include "geoPack3D.hpp"
#include "linreg.hpp"
//#include "octree.hpp"
//#include "linkCells.hpp"
#include "PeriodicNearestNeighbors.hpp"
#include "profiler.hpp"

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

  bool oldVersion;

  int nbBondsini;      ///< initial # of Bonds at start of Lagamine
  double porosityini;  ///< initial porosity at start of Lagamine
  int nbBonds;
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
  double density;    ///< Density of all particles
  double kn;         ///< Normal stiffness (for compression/tension, bonded or not)
  double kt;         ///< Tangential stiffness (bonded or not)
  double kr;         ///< Angular stiffness (only for bonded links)
  double dampRate;   ///< Viscous damping Rate -- in the range [0, 1[
  double mu;         ///< Coefficent of friction
  double mur;        ///< Coefficient of "angular-friction"
  double fcoh;       ///< Cohesion force (strictly negative)
  double zetaMax;    ///< Can be seen as "dn_rupture / dn_dammage_starts"
  double zetaInter;  ///< This is used for gate softening
  double Kratio;     ///< Ratio of particle stiffness over bond stiffness

  // Solid cohesion
  double fn0;      ///< Maximum normal force
  double ft0;      ///< Maximum tangential force
  double mom0;     ///< Maximum Torque
  double dn0;      ///< Maximum normal displacement
  double dt0;      ///< Maximum tangential displacement
  double drot0;    ///< Maximum angular rotation
  double powSurf;  ///< Power used in the breakage surface

  double rampRatio;
  double rampDuration;                       ///< linear laoding ramp between t = 0 and t = rampDuration

  std::string modelSoftening;                ///< Can be "linear", "gate" or "trainee"(default)
  std::function<double(double)> DzetaModel;  ///< Compute D as a function of zeta
  std::function<double(double)> zetaDModel;  ///< Compute zeta as a function of D

  // Other parameters
  int iconf;  ///< Current configuration ID
  // int enableSwitch;              ///< If non-null, enable the switch of particles from one boundary to the opposite
  int substractMeanVelocity;
  int limitHboxvelocity;
  double hboxLimitVel;
  int permamentGluer;            ///< If 1, contacts are permanently transformed to glued-point
  double numericalDampingCoeff;  ///< This is the so called Cundall damping
  int continuumContact;

  // Ctor
  PBC3Dbox();

  // Methods
  void showBanner();               ///< Displays a banner about the code
  void initOutputFiles();          ///< Opens output files that hold processed data (stress, fabric, etc.)
  void setSample();                ///< Creates a sample by asking questions to the user
  void velocityVerletStep();       ///< Makes a time increment with the velocity-Verlet scheme
  void integrate();                ///< Simulation flow
                                   ///< (iteratively make a time increment and check for updates or saving)
  void accelerations();            ///< Computes accelerations (both for particles and the periodic-cell)
  void computeForcesAndMoments();  ///< Computes forces and moments (and cell-stress)

  // Methods used for interaction of type 'bondedStateDam'
  double YieldFuncDam(double zeta, double Dn, double DtNorm, double DrotNorm);
  void setTraineeSoftening();
  void setLinearSoftening();
  void setGateSoftening();

  void printScreen(double elapsedTime);            ///< Prints usefull data on screen during computation
  void dataOutput();                               ///< Outputs usefull data during computation
  
	void updateNeighborList(double dmax);            ///< Updates the neighbor-list
  void updateNeighborList_brutForce(double dmax);  ///< Updates the neighbor-list
	
  void saveConf(const char* name);                 ///< Saves the current configuration in a file named 'name'
  void saveConf(int i);                            ///< Saves the current configuration in a file named confX, where X=i
  void loadConf(const char* name);                 ///< Loads a configuration from a file
  void clearMemory();                              ///< Clears the Particles and Interactions.
  void computeSampleData();                        ///< Computes a number of usefull data (Rmin, Rmax, Vsolid, etc.)
  void ActivateBonds(double epsiDist, int state);  ///< Replace contacts (with dn < epsiDist) by cemented bonds
  void RemoveBonds(double percentRemove, int StrategyId);  ///< A kind of global damage
  void freeze();                                           ///< Set all velocities (and accelerations) to zero

  // Methods specifically written for MPMbox (MPMxDEM coupling).
  void transform(mat9r& Finc, double macro_dt, int nstepMin, double rateAverage, double rateCriticalTimeStep,
                 mat9r& SigAvg);
  void applySwitchMatrix(mat9r& P);
  void ModularTransformation();

  // Methods specifically written for Lagamine (FEMxDEM coupling).
  // They are compatible with fortran (it's the reason why all parameters are pointers).
  void initLagamine(double Q[]);  ///< Kind of serialization solution to get the initial configuration from Lagamine
  void initLagamineSandstone(double Q[]);  ///< Gets the initial configuration from Lagamine
  void transform(double dFmoinsI[3][3], double* I, int* nstep, int* iana, double* pressure, double* sigRate);
  void hold(double* tol, int* nstepConv, int* nstepMax, int* iana, double* pressure, double* sigRate);
  void transform_and_hold(double dFmoinsI[3][3], double* I, double* tol, int* nstepConv, int* nstepMax, int* nstep,
                          int* iana, double* pressure, double* sigRate);
  void runSilently();  ///< Runs the simulation silently (without outputs) from time t to tmax
  void endLagamine(double Q[], double SIG[3][3]);           ///< Kind of serialization to get the data back
  void endLagamineSandstone(double Q[], double SIG[3][3]);  ///< Kind of serialization to get the data back
  void getOperatorKruyt(double L[6][6]);                    ///< Gets the operator proposed by Kruyt
  void getOperatorKruyt2(double L[9][9]);                   ///< Gets the operator proposed by Kruyt (version Kien)
  void getOperatorKruyt2b(double L[3][3][3][3]);
  void getOperatorKruyt3(double L[9][9]);                   ///< Gets the operator proposed by Kruyt
  ///< (version Kien avec sliding-contacts)

  void staticQualityData(double* ResMean, double* Res0Mean, double* fnMin,
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

  // balancing of stiffnesses
  double w_bond{0.5};
  double w_particle{0.5};

  int objectiveFriction{0};  //< activation of the objectivity for friction forces

 public:
  // Notice that these variables are publicly accessible,
  // but you need to ask for there pre-computation first.

  // Sample
  double Vsolid{0.0};  ///< Total volume of the particles
  double Vmin{0.0};    ///< Minimum volume of the particles
  double Vmax{0.0};    ///< Maximum volume of the particles
  double Vmean{0.0};   ///< Mean volume of the particles

  // Radii
  double Rmin{0.0};   ///< Minimum radius of the particles
  double Rmax{0.0};   ///< Maximum radius of the particles
  double Rmean{0.0};  ///< Mean radius of the particles

  // Normal forces
  double FnMin{0.0};   ///< Minimum normal force in the system
  double FnMax{0.0};   ///< Maximum normal force in the system
  double FnMean{0.0};  ///< Mean normal force in the system
  double ReducedPartDistMean{0.0};

  // Particle velocities
  double VelMin{0.0};   ///< Minimum velocity magnitude of the particles
  double VelMax{0.0};   ///< Maximum velocity magnitude of the particles
  double VelMean{0.0};  ///< Mean velocity magnitude of the particles
  double VelVar{0.0};   ///< Variance of velocity

  // Particles accelerations
  double AccMin{0.0};   ///< Minimum acceleration magnitude of the particles
  double AccMax{0.0};   ///< Maximum acceleration magnitude of the particles
  double AccMean{0.0};  ///< Mean acceleration magnitude of the particles
  double AccVar{0.0};   ///< Variance of acceleration
};

#endif /* end of include guard: PBC3D_SANDSTONE_HPP */
