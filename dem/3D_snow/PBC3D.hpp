#ifndef PBC3D_SNOW_HPP
#define PBC3D_SNOW_HPP

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

  // Other parameters
  int iconf;           ///< Current configuration ID
  int enableSwitch;    ///< If non-null, enable the switch of particles from one boundary to the opposite
  //int permamentGluer;  ///< If 1, contacts are permanently transformed to glued-point
  double numericalDampingCoeff;

  // Ctor
  PBC3Dbox();

  // Methods
  void showBanner();               ///< Displays a banner about the code
  void velocityVerletStep();       ///< Makes a time increment with the velocity-Verlet scheme
  void integrate();                ///< Simulation flow
                                   ///< (iteratively make a time increment and check for udates or saving)
  void accelerations();            ///< Computes accelerations (both for particles and the periodic-cell)
  void computeForcesAndMoments();  ///< Computes forces and moments (and cell-stress)

  void printScreen(double elapsedTime);  ///< Prints usefull data on screen during computation
  void getSubSpheres(vec3r& branch, size_t i, size_t j, std::vector<std::pair<size_t, size_t> >& duoIDs);
  void updateNeighborList(double dmax);  ///< Updates the neighbor-list
  void saveConf(int i);                  ///< Saves the current configuration in a file named confX, where X=i
  void loadConf(const char* name);       ///< Loads a configuration from a file
  void loadShapes();
  void clearMemory();  ///< Clears the Particles and Interactions.

 private:
  // Counters for the simulation flow
  double interVerletC;  ///< A counter for reconstruction of the neighbor list
  double interOutC;     ///< A counter for writting in output files
  double interConfC;    ///< A counter for writting conf files

  // time-step constants
  double dt_2;   ///< Half the time-step
  double dt2_2;  ///< Half the squared time-step

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

#endif /* end of include guard: PBC3D_SNOW_HPP */
