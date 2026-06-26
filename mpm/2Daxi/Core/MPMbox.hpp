#pragma once

//
// Header file for the MPMbox class and related declarations.
//
// This file contains the declaration of the MPMbox class, which is responsible
// for managing the simulation of material points within a grid using the Material Point Method (MPM).
// It includes the data structures and functions necessary for initializing the simulation,
// performing adaptive refinement, handling boundary conditions, and post-processing results.
//
// The class relies on several components such as nodes, elements, material points, obstacles,
// and various constitutive models to accurately simulate physical behaviors.
//
// The file also defines constants and includes necessary headers for the implementation of MPMbox.
//

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
  std::vector<Obstacle *> Obstacles; // List of rigid obstacles
  std::vector<Spy *> Spies;          // Spies for (post-)processing

  ShapeFunction *shapeFunction{nullptr};             // The shape functions
  OneStep *oneStep{nullptr};                         // Type of routine to be used
  std::map<std::string, ConstitutiveModel *> models; // The models

  std::vector<Scheduler *> Scheduled;  // the schedulled action to do
  std::vector<ControlMP> controlledMP; // force or velocity controls on some MP

  bool computationMode{true}; // true for computation; false for visualisation

  std::string result_folder{"."}; // The folder into which the result files will be saved
  bool planeStrain{false};        // Plane strain assumption
  grid Grid;                      // The fixed grid
  double tolmass{1e-6};           // Tolerance for the mass of a MaterialPoint

  vec2r gravity{0.0, 0.0}; // The gravity acceleration vector

  double finalTime{1.0}; // Time in seconds at which the simulation ends
  int step{0};           // The current step number
  int iconf{0};          // File number of the comming save
  int confPeriod{5000};  // Number of steps between conf files
  int proxPeriod{100};   // Number of steps between proximity check (rebuild the neighbor list)

  double dt{0.00001};        // Time increment
  double dtInitial{0.00001}; // Prescribed time increment
  double t{0.0};             // Current time

  double securDistFactor{2.0}; // Homothetic factor of shapes for proximity tests

  DataTable dataTable;
  size_t id_mu{0};
  size_t id_kn{0};
  size_t id_en2{0};
  size_t id_kt{0};
  size_t id_viscRate{0};
  size_t id_dn0{0};
  size_t id_dt0{0};

  bool splitting{false};           // Consistent splitting (vertical and horizontal)
  bool extremeShearing{false};     // Extreme shearing
  double extremeShearingval{0.0};  // Extreme shearing val is max ratio xx/xy or yy/yx that can be reached
  double splitCriterionValue{2.0}; // Elongation ratio for activating a split (whatever the direction)
  double shearLimit{0.0};          // max Fxy or Fyx value. After this F becomes Identity matrix
  int MaxSplitNumber{5};           // The maximum number of splits

  // integration scheme dissipation
  double ratioFLIP{0.95}; // barycenter coef for using PIC as damping
  bool activePIC{true};   // damping with PIC flag

  // CHCL is Computationally Hommogenized Constitutive Law
  // =====================================================
  struct {
    bool hasDoubleScale{false};             // to know if the computation involves CHCLs
    int minDEMstep{5};                      // minimum number of DEM time steps for linear regression of stress
    double rateAverage{0};                  // end-part of MPM time step used for stress averaging
    double limitTimeStepFactor{1e-3};       // ...
    double criticalDEMTimeStepFactor{0.01}; // ...
  } CHCL;

  std::vector<size_t> liveNodeNum; // list of node numbers being updated and used during each time step
                                   // It holds only the number of nodes concerned by the proximity of MP

  size_t number_MP_before_any_split; // used to check proximity if # of MP has changed
                                     // (some "unknown" points could enter the obstacle and suddenly be detected
                                     // once they are way inside)

  std::vector<std::string> BFLCommandStored; // stored command of Boudary Force Laws

  MPMbox();  // Ctor
  ~MPMbox(); // Dtor

  void ExplicitRegistrations();
  void showAppBanner();
  void setVerboseLevel(int v);
  void clean();
  void read(const char *name);
  void read(int num);
  void save(const char *name);
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
  void postProcess(std::vector<ProcessedDataMP> &MPPD);
};
