#ifndef MPMBOX_HPP
#define MPMBOX_HPP

/**
 * @file MPMbox.hpp
 * @brief Header file for the MPMbox class and related declarations.
 *
 * This file contains the declaration of the MPMbox class, which is responsible
 * for managing the simulation of material points within a grid using the Material Point Method (MPM).
 * It includes the data structures and functions necessary for initializing the simulation,
 * performing adaptive refinement, handling boundary conditions, and post-processing results.
 *
 * The class relies on several components such as nodes, elements, material points, obstacles,
 * and various constitutive models to accurately simulate physical behaviors.
 *
 * The file also defines constants and includes necessary headers for the implementation of MPMbox.
 *
 * @note This file requires C++17 standard and OpenMP for parallel processing.
 */

#include <algorithm>
#include <cstddef>
#include <ctime>
#include <exception>
#include <fstream>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <string>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

#include <stdlib.h>

//#define SPGLOG_HEADER_ONLY
//#define FMT_HEADER_ONLY
//#include "spdlog/fwd.h"

#define FMT_HEADER_ONLY
#include "fmtLogger.hpp"

// The linked DEM for HNL
#include "PBC3D.hpp"

// Headers in toofus
#include "DataTable.hpp"
#include "Mth.hpp"
#include "factory.hpp"
#include "fileTool.hpp"
#include "mat4.hpp"
#include "message.hpp"
#include "profiler.hpp"
#include "vec2.hpp"

// Headers that are part of MPMbox
#include "ControlMP.hpp"
#include "Element.hpp"
#include "Grid.hpp"
#include "Neighbor.hpp"
#include "Node.hpp"
#include "ProcessedDataMP.hpp"

struct MaterialPoint;
struct Obstacle;
struct Command;
struct Spy;
struct VtkOutput;
struct ShapeFunction;
struct OneStep;
struct ConstitutiveModel;
struct Scheduler;

class PBC3Dbox;

class MPMbox {
 public:
  std::vector<node> nodes;           // The nodes of the Eulerian grid
  std::vector<element> Elem;         // Quad-elements of the grid
  std::vector<MaterialPoint> MP;     // Material Points
  std::vector<Obstacle*> Obstacles;  // List of rigid obstacles
  std::vector<Spy*> Spies;           // Spies for (post-)processing

  ShapeFunction* shapeFunction;                      // The shape functions
  OneStep* oneStep;                                  // Type of routine to be used
  std::map<std::string, ConstitutiveModel*> models;  // The models

  std::vector<Scheduler*> Scheduled;    // the schedulled action to do
  std::vector<ControlMP> controlledMP;  // force or velocity controls on some MP

  bool computationMode{true};

  std::string result_folder;  // The folder into which the result files will be saved
  bool planeStrain;           // Plane strain assumption (default value is false)
  grid Grid;                  // The fixed grid
  double tolmass;             // Tolerance for the mass of a MaterialPoint

  vec2r gravity;  // The gravity acceleration vector

  double finalTime;  // Time in seconds at which the simulation ends
  int step;          // The current step number
  int iconf;         // File number of the comming save
  int confPeriod;    // Number of steps between conf files
  int proxPeriod;    // Number of steps between proximity check (rebuild the neighbor list)

  double dt;         // Time increment
  double dtInitial;  // Prescribed time increment
  double t;          // Current time

  double securDistFactor;  // Homothetic factor of shapes for proximity tests

  DataTable dataTable;
  size_t id_mu, id_kn, id_en2, id_kt, id_viscRate, id_dn0, id_dt0;

  bool splitting;              // Consistent splitting (vertical and horizontal)
  bool extremeShearing;        // Extreme shearing
  double extremeShearingval;   // Extreme shearing val refers to the max ratio xx/xy or yy/yx that can be reached
  double splitCriterionValue;  // Elongation ratio for activating a split (whatever the direction)
  double shearLimit;           // max Fxy or Fyx value. After this F becomes Identity matrix
  int MaxSplitNumber;          // The maximum number of splits

  // integration scheme dissipation
  double ratioFLIP;  // barycenter coef for using PIC as damping
  bool activePIC;    // damping with PIC flag ***** rename blended_PIC_FLIP

  struct {
    bool hasDoubleScale;  // to know if the computation involves CHCLs
    int minDEMstep;       // minimum number of DEM time steps for linear regression of stress
    double rateAverage;   // end-part of MPM time step used for stress averaging
    double limitTimeStepFactor;
    double criticalDEMTimeStepFactor;
  } CHCL;

  std::vector<size_t> liveNodeNum;  // list of node numbers being updated and used during each time step
                                    // It holds only the number of nodes concerned by the proximity of MP

  size_t number_MP_before_any_split;  // used to check proximity if # of MP has changed
                                      // (some "unknown" points could enter the obstacle and suddenly be detected
                                      // once they are way inside)

  MPMbox();   // Ctor
  ~MPMbox();  // Dtor

  void ExplicitRegistrations();
  void showAppBanner();
  void setVerboseLevel(int v);
  void clean();
  void read(const char* name);
  void read(int num);
  void save(const char* name);
  void save(int num);
  void checkProximity();
  void init();

  void MPinGridCheck();
  void convergenceConditions();
  void run();

  // Functions called in OneStep
  void updateVelocityGradient();
  void limitTimeStepForDEM();
  void updateTransformationGradient();
  void plannedRemovalObstacle();
  void plannedRemovalMP();
  void adaptativeRefinement();

  // postprocessing functions
  void postProcess(std::vector<ProcessedDataMP>& MPPD);
};

#endif /* end of include guard: MPMBOX_HPP */
