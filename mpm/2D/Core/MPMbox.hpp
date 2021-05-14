#ifndef MPMBOX_HPP
#define MPMBOX_HPP

#include <algorithm>
#include <cstddef>
#include <ctime>
#include <exception>
#include <fstream>
#include <map>
#include <numeric>
#include <set>
#include <string>
#include <vector>

#if defined(_OPENMP)
#include <omp.h>
#endif

#include <stdlib.h>

// Headers in common
#include "DataTable.hpp"
#include "Mth.hpp"
#include "PBC3D.hpp"
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

struct MaterialPoint;
struct Obstacle;
struct Command;
struct Spy;
struct VtkOutput;
struct ShapeFunction;
struct OneStep;
struct ConstitutiveModel;
class PBC3Dbox;

struct MPMbox {
  std::vector<node> nodes;        // The nodes of the Eulerian grid
  std::vector<int> liveNodeNum;   // list of node numbers being used during each time step
  std::vector<element> Elem;      // Quad-elements of the grid
  std::vector<MaterialPoint> MP;  // Material Points
  std::vector<PBC3Dbox> PBC;      // DEM simulation containers

  std::ofstream logFile;
  std::ofstream logFile2;

  std::vector<Obstacle*> Obstacles;    // List of rigid obstacles
  std::vector<Spy*> Spies;             // Spies for (post-)processing
  std::vector<VtkOutput*> VtkOutputs;  // Options to populate the vtk files

  ShapeFunction* shapeFunction;                      // The shape functions
  OneStep* oneStep;                                  // Type of routine to be used
  std::map<std::string, ConstitutiveModel*> models;  // The models

  std::string result_folder;  // The folder into which the result files will be saved
  std::string oneStepType;    // Name Identifier of the step algorithm
  bool planeStrain;  // Plane strain assumption (plane stress if false, default)
  grid Grid;         // The fixed grid
  double tolmass;    // Tolerance for the mass of a MaterialPoint
  vec2r gravity;     // The gravity acceleration vector

  int nstep;         // Number of steps to be done
  double finalTime;  // Time in seconds at which the simulation ends (to replace nstep)
  int step;          // The current step
  int vtkPeriod;     // Number of steps between vtk files
  int proxPeriod;    // Number of steps between proximity check (rebuild the neighbor list)
  double dt;         // Time increment
  double t;          // Current time
  mat9r V0;

  double securDistFactor;  // Homothetic factor of shapes for proximity tests

  DataTable dataTable;
  size_t id_mu, id_kn, id_en2, id_kt, id_viscRate;

  bool splitting;                   // Consistent splitting (vertical and horizontal)
  bool extremeShearing;             // Extreme shearing
  double extremeShearingval;        // Extreme shearing val refers to the max ratio xx/xy or yy/yx that can be reached
  bool splittingMore;               // Inconsistent splitting (diagonals)
  double splitCriterionValue;       // Elongation ratio for activating a split (whatever the direction)
  double shearLimit;                // max Fxy or Fyx value. After this F becomes Identity matrix
  int MaxSplitNumber;               // The maximum number of splits
  double NumericalDissipation;      // value of alpha for pfc dissipation. the closer to 0, the more it dissipates
  bool activeNumericalDissipation;  // Flag for activating the numerical dissipation
  size_t number_MP;                 // used to check proximity if # of MP has changed
                                    // (some "unknown" points could enter the obstacle and suddenly be detected
                                    // once they are way inside)

  std::vector<vec2r> surfacePoints;
  bool parallelogramMP;  // Save MP as parallelogram (true/false)
  std::vector<double> MPmassIncrement;

  MPMbox();   // Ctor
  ~MPMbox();  // Dtor

  void showAppBanner();
  void read(const char* name);
  void save(const char* name);
  void save(int num);
  void setDefaultVtkOutputs();
  void checkNumericalDissipation();
  void checkProximity();
  void save_vtk_grid();
  void save_vtk_obst(const char* base, int num);
  void save_vtk_surface();
  void save_vtk(const char* base, int num);
  void init(/*const char* name, const char* dconf*/);
  void MPinGridCheck();
  void cflCondition();
  void run();

  // Functions called in OneStep
  void boundaryConditions();
  void updateTransformationGradient();
  void adaptativeRefinement();
  void adaptativeRefinementMore();
  void weightIncrement();

  void save_state(const char* base, int num);
};

#endif /* end of include guard: MPMBOX_HPP */
