#ifndef MPMBOX_HPP
#define MPMBOX_HPP

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

#define SPGLOG_HEADER_ONLY
#define FMT_HEADER_ONLY
//#include "spdlog/sinks/stdout_color_sinks.h"
//#include "spdlog/spdlog.h"
#include "spdlog/fwd.h"

// The linked DEM for HNL
#include "PBC3D.hpp"

// Headers in toofus
#include "DataTable.hpp"
#include "Mth.hpp"
#include "factory.hpp"
#include "fileTool.hpp"
#include "mat4.hpp"
#include "message.hpp"
#include "vec2.hpp"

// Headers that are part of MPMbox
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
class PBC3Dbox;

class MPMbox {
 public:
  std::vector<node> nodes;        // The nodes of the Eulerian grid
  std::vector<element> Elem;      // Quad-elements of the grid
  std::vector<MaterialPoint> MP;  // Material Points

  std::vector<Obstacle*> Obstacles;  // List of rigid obstacles
  std::vector<Spy*> Spies;           // Spies for (post-)processing

  ShapeFunction* shapeFunction;                      // The shape functions
  OneStep* oneStep;                                  // Type of routine to be used
  std::map<std::string, ConstitutiveModel*> models;  // The models

  std::string result_folder;  // The folder into which the result files will be saved
  bool planeStrain;           // Plane strain assumption (default value is false)
  grid Grid;                  // The fixed grid
  double tolmass;             // Tolerance for the mass of a MaterialPoint

  vec2r gravity;       // The gravity acceleration vector
  vec2r gravity_max;   // ramp gravity acceleration vector
  vec2r gravity_incr;  // increment gravity acceleration vector
  bool ramp;           // ramp boolean

  double boundary_layer;  // enlarge the contact zone

  double finalTime;  // Time in seconds at which the simulation ends
  int step;          // The current step number
  int iconf;         // File number of the comming save
  int confPeriod;    // Number of steps between conf files
  int proxPeriod;    // Number of steps between proximity check (rebuild the neighbor list)

  int DEMPeriod;  // Number of spatial steps for DEM sampling FIXME: will be replaced by something that is able to
                  // manage different solutions

  double dt;         // Time increment
  double dtInitial;  // Prescribed time increment
  double t;          // Current time

  // scheduled removal of an obstacle:
  // double deltime;    // time to delete an obstacle
  // int delnumber;     // osbsacle number to be deleted

  struct {
    int groupNumber;  // osbsacle number to be deleted
    double time;      // time to remove an obstacle

  } ObstaclePlannedRemoval;

  double securDistFactor;  // Homothetic factor of shapes for proximity tests

  DataTable dataTable;
  size_t id_mu, id_kn, id_en2, id_kt, id_viscRate;

  bool splitting;              // Consistent splitting (vertical and horizontal)
  bool extremeShearing;        // Extreme shearing
  double extremeShearingval;   // Extreme shearing val refers to the max ratio xx/xy or yy/yx that can be reached
  double splitCriterionValue;  // Elongation ratio for activating a split (whatever the direction)
  double shearLimit;           // max Fxy or Fyx value. After this F becomes Identity matrix
  int MaxSplitNumber;          // The maximum number of splits

  double ViscousDissipation;  // drag-like dissipation 0 means no dissipation FIXME: ne semble plus utilisé

  // transient dissipation  FIXME: déactivé ??? sinon mettre dans une struct
  double NumericalDissipation;      // value of alpha for pfc dissipation. the closer to 0, the more it dissipates
  double minVd;                     // value of velocity norm beyond which dissipation is desactivated
  double EndNd;                     // time ending numerical dissipation
  bool activeNumericalDissipation;  // Flag for activating the numerical dissipation

  // integration scheme dissipation
  double ratioFLIP;  // barycenter coef for using PIC as damping
  bool activePIC;    // damping with PIC flag FIXME: public ???
  double timePIC;    // end of damping PIC

  bool dispacc;  // activates doubles saves FIXME: pas trop compris l'utilité

  struct {
    bool hasDoubleScale;
  } NHL;

  int DEMstep;           // minimal number of DEM time steps FIXME: rename and move into NHL
  double lengthAverage;  // proportion of DEM aveaging FIXME: rename and move into NHL

  std::vector<int> liveNodeNum;  // list of node numbers being updated and used during each time step
                                 // It holds only the number of nodes concerned by the proximity of MP

  size_t number_MP;  // used to check proximity if # of MP has changed
                     // (some "unknown" points could enter the obstacle and suddenly be detected
                     // once they are way inside)

  std::shared_ptr<spdlog::logger> console;

  MPMbox();   // Ctor
  ~MPMbox();  // Dtor

  void showAppBanner();
  void setVerboseLevel(int v);
  void clean();
  void read(const char* name);
  void save(const char* name);
  void save(int num);
  void checkNumericalDissipation(double minVd, double EndNd);
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
  void adaptativeRefinement();
  // void weightIncrement();

  // postprocessing functions
  void postProcess(std::vector<ProcessedDataMP>& MPPD);
};

#endif /* end of include guard: MPMBOX_HPP */
