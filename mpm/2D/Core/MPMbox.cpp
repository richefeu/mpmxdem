#include "MPMbox.hpp"

#include "BoundaryForceLaw/BoundaryForceLaw.hpp"
#include "BoundaryForceLaw/frictionalNormalRestitution.hpp"
#include "BoundaryForceLaw/frictionalViscoElastic.hpp"

#include "Commands/Command.hpp"
#include "Commands/add_MP_ShallowPath.hpp"
#include "Commands/move_MP.hpp"
#include "Commands/new_set_grid.hpp"
#include "Commands/reset_model.hpp"
#include "Commands/select_controlled_MP.hpp"
#include "Commands/select_tracked_MP.hpp"
#include "Commands/set_BC_column.hpp"
#include "Commands/set_BC_line.hpp"
#include "Commands/set_K0_stress.hpp"
#include "Commands/set_MP_grid.hpp"
#include "Commands/set_MP_polygon.hpp"
#include "Commands/set_node_grid.hpp"
#include "Commands/set_uniform_pressure.hpp"

#include "ConstitutiveModels/CHCL_DEM.hpp"
#include "ConstitutiveModels/ConstitutiveModel.hpp"
#include "ConstitutiveModels/HookeElasticity.hpp"
#include "ConstitutiveModels/KelvinVoigt.hpp"
#include "ConstitutiveModels/MohrCoulomb.hpp"
#include "ConstitutiveModels/SinfoniettaClassica.hpp"
#include "ConstitutiveModels/SinfoniettaCrush.hpp"
#include "ConstitutiveModels/VonMisesElastoPlasticity.hpp"

#include "Obstacles/Circle.hpp"
#include "Obstacles/Line.hpp"
#include "Obstacles/Obstacle.hpp"
#include "Obstacles/Polygon.hpp"

#include "OneStep/ModifiedLagrangian.hpp"
#include "OneStep/OneStep.hpp"
#include "OneStep/UpdateStressFirst.hpp"
#include "OneStep/UpdateStressLast.hpp"

#include "ShapeFunctions/BSpline.hpp"
#include "ShapeFunctions/Linear.hpp"
#include "ShapeFunctions/RegularQuadLinear.hpp"
#include "ShapeFunctions/ShapeFunction.hpp"

#include "Spies/ElasticBeamDev.hpp"
#include "Spies/EnergyBalance.hpp"
#include "Spies/MPTracking.hpp"
#include "Spies/MeanStress.hpp"
#include "Spies/ObstacleTracking.hpp"
#include "Spies/Spy.hpp"
#include "Spies/Work.hpp"

#include "Schedulers/GravityRamp.hpp"
#include "Schedulers/MoveObstacle.hpp"
#include "Schedulers/PICDissipation.hpp"
#include "Schedulers/PICDissipationByPIC.hpp"
#include "Schedulers/ReactivateCHCLBonds.hpp"
#include "Schedulers/RemoveMaterialPoint.hpp"
#include "Schedulers/RemoveObstacle.hpp"

#include "Core/MaterialPoint.hpp"

#include "Mth.hpp"

#include <list>

//
// Constructor of the MPMbox class
//
// Initializes the MPMbox with default values for its fields.
//
// The constructor also calls the ExplicitRegistrations() function.
//
MPMbox::MPMbox() {
  id_kn       = dataTable.add("kn");
  id_kt       = dataTable.add("kt");
  id_en2      = dataTable.add("en2");
  id_mu       = dataTable.add("mu");
  id_viscRate = dataTable.add("viscRate");
  id_dn0      = dataTable.add("dn0");
  id_dt0      = dataTable.add("dt0");

  ExplicitRegistrations();
}

//
// Destructor of the MPMbox class.
//
// This is the default destructor of the MPMbox class, which is
// responsible for cleaning up the allocated memory.
//
// The destructor just calls the clean() method, which is
// responsible for freeing the memory allocated during the
// initialization of the MPMbox object. This is important to
// prevent memory leaks.
//
MPMbox::~MPMbox() {
  clean();
}

//
// Displays the application banner.
//
// This function outputs a stylized banner to the console,
// representing the application's name or logo using ASCII art.
// It adds a visual separator before and after the banner
// for better readability.
//
void MPMbox::showAppBanner() {
  std::cout << std::endl;
  std::cout << "    _/      _/  _/_/_/    _/      _/  _/                         " << std::endl;
  std::cout << "   _/_/  _/_/  _/    _/  _/_/  _/_/  _/_/_/      _/_/    _/    _/" << std::endl;
  std::cout << "  _/  _/  _/  _/_/_/    _/  _/  _/  _/    _/  _/    _/    _/_/   " << std::endl;
  std::cout << " _/      _/  _/        _/      _/  _/    _/  _/    _/  _/    _/  " << std::endl;
  std::cout << "_/      _/  _/        _/      _/  _/_/_/      _/_/    _/    _/   " << std::endl;
  std::cout << std::endl;
}

//
// Registers all the necessary classes to the factories.
//
// This is a static method that registers all the necessary classes to the factories.
// It is called by the constructor of the MPMbox class.
//
// This method is responsible for registering all the necessary classes to the factories,
// which are used later on in the code to create objects of the registered classes.
// The classes are registered by calling the RegisterFactoryFunction method of the
// corresponding factory, and providing a lambda function that returns an instance of
// the class to be registered.
//
void MPMbox::ExplicitRegistrations() {

  // BoundaryForceLaw ==========
  Factory<BoundaryForceLaw, std::string>::Instance()->RegisterFactoryFunction(
      "frictionalNormalRestitution", [](void) -> BoundaryForceLaw * { return new frictionalNormalRestitution(); });
  Factory<BoundaryForceLaw, std::string>::Instance()->RegisterFactoryFunction(
      "frictionalViscoElastic", [](void) -> BoundaryForceLaw * { return new frictionalViscoElastic(); });

  // Command ===================
  Factory<Command, std::string>::Instance()->RegisterFactoryFunction(
      "add_MP_ShallowPath", [](void) -> Command * { return new add_MP_ShallowPath(); });
  Factory<Command, std::string>::Instance()->RegisterFactoryFunction("move_MP",
                                                                     [](void) -> Command * { return new move_MP(); });
  Factory<Command, std::string>::Instance()->RegisterFactoryFunction(
      "new_set_grid", [](void) -> Command * { return new new_set_grid(); });
  Factory<Command, std::string>::Instance()->RegisterFactoryFunction(
      "reset_model", [](void) -> Command * { return new reset_model(); });
  Factory<Command, std::string>::Instance()->RegisterFactoryFunction(
      "set_BC_column", [](void) -> Command * { return new set_BC_column(); });
  Factory<Command, std::string>::Instance()->RegisterFactoryFunction(
      "set_BC_line", [](void) -> Command * { return new set_BC_line(); });
  Factory<Command, std::string>::Instance()->RegisterFactoryFunction(
      "set_K0_stress", [](void) -> Command * { return new set_K0_stress(); });
  Factory<Command, std::string>::Instance()->RegisterFactoryFunction(
      "set_MP_grid", [](void) -> Command * { return new set_MP_grid(); });
  Factory<Command, std::string>::Instance()->RegisterFactoryFunction(
      "set_MP_polygon", [](void) -> Command * { return new set_MP_polygon(); });
  Factory<Command, std::string>::Instance()->RegisterFactoryFunction(
      "set_node_grid", [](void) -> Command * { return new set_node_grid(); });
  Factory<Command, std::string>::Instance()->RegisterFactoryFunction(
      "select_tracked_MP", [](void) -> Command * { return new select_tracked_MP(); });
  Factory<Command, std::string>::Instance()->RegisterFactoryFunction(
      "select_controlled_MP", [](void) -> Command * { return new select_controlled_MP(); });
  Factory<Command, std::string>::Instance()->RegisterFactoryFunction(
      "set_uniform_pressure", [](void) -> Command * { return new set_uniform_pressure(); });

  // ConstitutiveModel =========
  Factory<ConstitutiveModel, std::string>::Instance()->RegisterFactoryFunction(
      "CHCL_DEM", [](void) -> ConstitutiveModel * { return new CHCL_DEM(); });
  Factory<ConstitutiveModel, std::string>::Instance()->RegisterFactoryFunction(
      "HookeElasticity", [](void) -> ConstitutiveModel * { return new HookeElasticity(); });
  Factory<ConstitutiveModel, std::string>::Instance()->RegisterFactoryFunction(
      "KelvinVoigt", [](void) -> ConstitutiveModel * { return new KelvinVoigt(); });
  Factory<ConstitutiveModel, std::string>::Instance()->RegisterFactoryFunction(
      "MohrCoulomb", [](void) -> ConstitutiveModel * { return new MohrCoulomb(); });
  Factory<ConstitutiveModel, std::string>::Instance()->RegisterFactoryFunction(
      "VonMisesElastoPlasticity", [](void) -> ConstitutiveModel * { return new VonMisesElastoPlasticity(); });
  Factory<ConstitutiveModel, std::string>::Instance()->RegisterFactoryFunction(
      "SinfoniettaClassica", [](void) -> ConstitutiveModel * { return new SinfoniettaClassica(); });
  Factory<ConstitutiveModel, std::string>::Instance()->RegisterFactoryFunction(
      "SinfoniettaCrush", [](void) -> ConstitutiveModel * { return new SinfoniettaCrush(); });

  // Obstacle ==================
  Factory<Obstacle, std::string>::Instance()->RegisterFactoryFunction("Circle",
                                                                      [](void) -> Obstacle * { return new Circle(); });
  Factory<Obstacle, std::string>::Instance()->RegisterFactoryFunction("Line",
                                                                      [](void) -> Obstacle * { return new Line(); });
  Factory<Obstacle, std::string>::Instance()->RegisterFactoryFunction("Polygon",
                                                                      [](void) -> Obstacle * { return new Polygon(); });

  // OneStep ===================
  Factory<OneStep, std::string>::Instance()->RegisterFactoryFunction(
      "ModifiedLagrangian", [](void) -> OneStep * { return new ModifiedLagrangian(); });
  Factory<OneStep, std::string>::Instance()->RegisterFactoryFunction(
      "UpdateStressFirst", [](void) -> OneStep * { return new UpdateStressFirst(); });
  Factory<OneStep, std::string>::Instance()->RegisterFactoryFunction(
      "UpdateStressLast", [](void) -> OneStep * { return new UpdateStressLast(); });

  // ShapeFunction =============
  Factory<ShapeFunction, std::string>::Instance()->RegisterFactoryFunction(
      "BSpline", [](void) -> ShapeFunction * { return new BSpline(); });
  Factory<ShapeFunction, std::string>::Instance()->RegisterFactoryFunction(
      "Linear", [](void) -> ShapeFunction * { return new Linear(); });
  Factory<ShapeFunction, std::string>::Instance()->RegisterFactoryFunction(
      "RegularQuadLinear", [](void) -> ShapeFunction * { return new RegularQuadLinear(); });

  // Scheduler ==================
  Factory<Scheduler, std::string>::Instance()->RegisterFactoryFunction(
      "GravityRamp", [](void) -> Scheduler * { return new GravityRamp(); });
  Factory<Scheduler, std::string>::Instance()->RegisterFactoryFunction(
      "MoveObstacle", [](void) -> Scheduler * { return new MoveObstacle(); });
  Factory<Scheduler, std::string>::Instance()->RegisterFactoryFunction(
      "PICDissipation", [](void) -> Scheduler * { return new PICDissipation(); });
  Factory<Scheduler, std::string>::Instance()->RegisterFactoryFunction(
      "PICDissipationByPIC", [](void) -> Scheduler * { return new PICDissipationByPIC(); });
  // Alias names: keep backward compatibility while providing clearer semantics.
  Factory<Scheduler, std::string>::Instance()->RegisterFactoryFunction(
      "PICDissipationPICRatio", [](void) -> Scheduler * { return new PICDissipationByPIC(); });
  Factory<Scheduler, std::string>::Instance()->RegisterFactoryFunction(
      "RemoveObstacle", [](void) -> Scheduler * { return new RemoveObstacle(); });
  Factory<Scheduler, std::string>::Instance()->RegisterFactoryFunction(
      "RemoveMaterialPoint", [](void) -> Scheduler * { return new RemoveMaterialPoint(); });
  Factory<Scheduler, std::string>::Instance()->RegisterFactoryFunction(
      "ReactivateCHCLBonds", [](void) -> Scheduler * { return new ReactivateCHCLBonds(); });

  // Spy ========================
  Factory<Spy, std::string>::Instance()->RegisterFactoryFunction("ObstacleTracking",
                                                                 [](void) -> Spy * { return new ObstacleTracking(); });
  Factory<Spy, std::string>::Instance()->RegisterFactoryFunction("Work", [](void) -> Spy * { return new Work(); });
  Factory<Spy, std::string>::Instance()->RegisterFactoryFunction("EnergyBalance",
                                                                 [](void) -> Spy * { return new EnergyBalance(); });
  Factory<Spy, std::string>::Instance()->RegisterFactoryFunction("MeanStress",
                                                                 [](void) -> Spy * { return new MeanStress(); });
  Factory<Spy, std::string>::Instance()->RegisterFactoryFunction("MPTracking",
                                                                 [](void) -> Spy * { return new MPTracking(); });
  Factory<Spy, std::string>::Instance()->RegisterFactoryFunction("ElasticBeamDev",
                                                                 [](void) -> Spy * { return new ElasticBeamDev(); });
}

//
// Sets the verbosity level for the logger
//
// The verbosity level corresponds to the following levels of logging:
//  - 0: off
//  - 1: critical
//  - 2: error
//  - 3: warning
//  - 4: info
//  - 5: debug
//  - 6: trace
//
// If the given verbosity level is not recognized, the logger will default to info level.
//
void MPMbox::setVerboseLevel(int v) {
  switch (v) {
  case 6:
    Logger::setLevel(LogLevel::trace);
    break;
  case 5:
    Logger::setLevel(LogLevel::debug);
    break;
  case 4:
    Logger::setLevel(LogLevel::info);
    break;
  case 3:
    Logger::setLevel(LogLevel::warn);
    break;
  case 2:
    Logger::setLevel(LogLevel::error);
    break;
  case 1:
    Logger::setLevel(LogLevel::critical);
    break;
  case 0:
    Logger::setLevel(LogLevel::off);
    break;
  default:
    Logger::setLevel(LogLevel::info);
    break;
  }
}

//
// Cleans up the MPMbox object by releasing allocated resources.
//
// This method clears the internal data structures used by the MPMbox,
// including nodes, elements, material points (MP), obstacles, and models.
// It deletes dynamically allocated memory for obstacles and constitutive
// models to prevent memory leaks. After calling this function, the MPMbox
// object is reset to an empty state.
//
void MPMbox::clean() {
  nodes.clear();
  Elem.clear();
  MP.clear();

  for (size_t i = 0; i < Obstacles.size(); i++) { delete (Obstacles[i]); }
  Obstacles.clear();

  std::map<std::string, ConstitutiveModel *>::iterator itModel;
  for (itModel = models.begin(); itModel != models.end(); ++itModel) { delete itModel->second; }
  models.clear();
}

//
// Reads a MPMbox object from a file.
//
// This function reads a MPMbox object from a file, which must be a text file
// containing the information about the nodes, elements, material points, obstacles,
// models, and other parameters of the MPMbox. The file format is specific to
// the MPMbox library and is described in the user manual.
//
void MPMbox::read(const char *name) {
  std::ifstream file(name);
  if (!file) {
    Logger::warn("@MPMbox::read, cannot open file {}", name);
    return;
  }

  BFLCommandStored.clear();

  std::string token;
  file >> token;
  while (file) {
    if (token[0] == '/' || token[0] == '#' || token[0] == '!') {
      getline(file, token);
    } else if (token == "result_folder") {
      file >> result_folder;
      // If result_folder does not exist, it is created
      fileTool::create_folder(result_folder);
    } else if (token == "oneStepType") {
      std::string typeOneStep;
      file >> typeOneStep;
      if (oneStep) {
        delete oneStep;
        oneStep = nullptr;
      }
      oneStep = Factory<OneStep>::Instance()->Create(typeOneStep);
      if (oneStep == nullptr) {
        Logger::critical("@MPMbox::read, oneStepType '{}' is unknown", typeOneStep);
        Logger::critical("  Known types: ModifiedLagrangian, UpdateStressFirst, UpdateStressLast");
        exit(EXIT_FAILURE);
      }
    } else if (token == "planeStrain") {
      planeStrain = true;
    } else if (token == "tolmass") {
      file >> tolmass;
    } else if (token == "gravity") {
      file >> gravity;
    } else if (token == "finalTime") {
      file >> finalTime;
    } else if (token == "confPeriod") {
      file >> confPeriod;
    } else if (token == "proxPeriod") {
      file >> proxPeriod;
    } else if (token == "securDistFactor") {
      file >> securDistFactor;
    } else if (token == "dt") {
      file >> dt;
    } else if (token == "t") {
      file >> t;
    } else if (token == "enablePIC") {
      double ratioPIC;
      file >> ratioPIC;
      if (ratioPIC < 0.0 || ratioPIC > 1.0) {
        Logger::warn("The PIC ratio (here {}) should be set in range 0 to 1", ratioPIC);
      }
      ratioFLIP = 1.0 - ratioPIC;
      activePIC = true;
    } else if (token == "disablePIC") {
      activePIC = false;
    } else if (token == "splitting") {
      file >> splitting;
    } else if (token == "splittingExtremeShearing") {
      file >> extremeShearing >> extremeShearingval;
    } else if (token == "splitCriterionValue") {
      file >> splitCriterionValue;
    } else if (token == "shearLimit") {
      file >> shearLimit;
    } else if (token == "MaxSplitNumber") {
      file >> MaxSplitNumber;
    } else if (token == "demavg") { // kept for compatibility
      file >> CHCL.minDEMstep >> CHCL.rateAverage;
    } else if (token == "CHCL.minDEMstep") {
      file >> CHCL.minDEMstep;
    } else if (token == "CHCL.rateAverage") {
      file >> CHCL.rateAverage;
    } else if (token == "CHCL.limitTimeStepFactor") {
      file >> CHCL.limitTimeStepFactor;
    } else if (token == "CHCL.criticalDEMTimeStepFactor") {
      file >> CHCL.criticalDEMTimeStepFactor;
    } else if (token == "set") {
      std::string param;
      size_t g1, g2; // g1 corresponds to MPgroup and g2 to obstacle group
      double value;
      file >> param >> g1 >> g2 >> value;
      // DataTable::set silently CREATES the parameter when the name is unknown,
      // so a typo used to be accepted without a word while the intended
      // parameter stayed at zero. The seven usable names are the ones added by
      // the constructor of MPMbox.
      if (!dataTable.exists(param)) {
        Logger::critical("@MPMbox::read, unknown interaction parameter in 'set {} {} {} {}'", param, g1, g2, value);
        Logger::critical("  Known parameters: kn, kt, en2, mu, viscRate, dn0, dt0");
        exit(EXIT_FAILURE);
      }
      dataTable.set(param, g1, g2, value);
    } else if (token == "prescribedVelocity") { // TODO (V) -> Move it as a command
      int groupNb;
      vec2r prescribedVel;
      file >> groupNb >> prescribedVel;
      for (size_t p = 0; p < MP.size(); p++) {
        // improve because there are vectors containing the groups already
        if (groupNb == MP[p].groupNb) MP[p].vel = prescribedVel;
      }
    } else if (token == "ShapeFunction") {
      std::string shapeFunctionName;
      file >> shapeFunctionName;

      if (shapeFunction) {
        delete shapeFunction;
        shapeFunction = nullptr;
      }
      shapeFunction = Factory<ShapeFunction>::Instance()->Create(shapeFunctionName);
      if (shapeFunction == nullptr) {
        Logger::critical("@MPMbox::read, ShapeFunction '{}' is unknown", shapeFunctionName);
        Logger::critical("  Known shape functions: Linear, RegularQuadLinear, BSpline");
        exit(EXIT_FAILURE);
      }
    } else if (token == "model") {
      std::string modelName, modelID;
      file >> modelID >> modelName;
      ConstitutiveModel *CM = Factory<ConstitutiveModel>::Instance()->Create(modelID);
      if (CM != nullptr) {
        models[modelName] = CM;
        CM->key           = modelName;
        CM->box           = this;
        CM->read(file);
      } else {
        Logger::warn("mode {} is unknown!", modelID);
      }
    } else if (token == "Obstacle") {
      std::string obsName;
      file >> obsName;
      Obstacle *obs = Factory<Obstacle>::Instance()->Create(obsName);
      if (obs != nullptr) {
        obs->read(file);
        Obstacles.push_back(obs);
      } else {
        Logger::warn("Obstacle {} is unknown!", obsName);
      }
    } else if (token == "BoundaryForceLaw") {
      // This has to be defined after defining the obstacles
      if (Obstacles.empty()) { Logger::warn("You try to define BoundaryForceLaw BEFORE any Obstacle is set!"); }
      std::string boundaryName;
      int obstacleGroup;
      file >> boundaryName >> obstacleGroup;

      char StoredCommand[256];
      snprintf(StoredCommand, 256, "BoundaryForceLaw %s %d", boundaryName.c_str(), obstacleGroup);
      BFLCommandStored.push_back(std::string(StoredCommand));

      // A first instance is created only to check the name: an unknown name
      // used to give a null pointer, quietly assigned to the obstacles and
      // dereferenced at the first time step.
      BoundaryForceLaw *probe = Factory<BoundaryForceLaw>::Instance()->Create(boundaryName);
      if (probe == nullptr) {
        Logger::critical("@MPMbox::read, BoundaryForceLaw '{}' is unknown", boundaryName);
        Logger::critical("  Known laws: frictionalNormalRestitution, frictionalViscoElastic, "
                         "frictionalViscoElastofragile");
        exit(EXIT_FAILURE);
      }
      delete probe;

      // One instance per obstacle. Sharing a single one between the obstacles
      // of a group would make the ownership ambiguous, and the default law set
      // by the constructor of Obstacle has to be released.
      size_t nbAssigned = 0;
      for (size_t o = 0; o < Obstacles.size(); o++) {
        if (Obstacles[o]->group == obstacleGroup) {
          delete Obstacles[o]->boundaryForceLaw;
          Obstacles[o]->boundaryForceLaw = Factory<BoundaryForceLaw>::Instance()->Create(boundaryName);
          nbAssigned++;
        }
      }
      if (nbAssigned == 0) {
        Logger::warn("@MPMbox::read, 'BoundaryForceLaw {} {}': no obstacle belongs to group {}", boundaryName,
                     obstacleGroup, obstacleGroup);
      }
    } else if (token == "ObstacleNeighbors") {
      // This has to be defined after defining the obstacles
      if (Obstacles.empty()) { Logger::warn("You try to define ObstacleNeighbors BEFORE any Obstacle is set!"); }
      if (MP.empty()) { Logger::warn("You try to define ObstacleNeighbors BEFORE any MP is set!"); }

      size_t nbNeighbors;
      Neighbor N;
      for (size_t o = 0; o < Obstacles.size(); o++) {
        file >> nbNeighbors;
        Obstacles[o]->Neighbors.clear();
        Obstacles[o]->force.reset();
        Obstacles[o]->acc.reset();
        vec2r Nvec;
        vec2r Tvec;
        for (size_t n = 0; n < nbNeighbors; n++) {
          file >> N.PointNumber >> N.fn >> N.dn >> N.ft >> N.dt >> N.sigma_n;
          Obstacles[o]->getContactFrame(MP[N.PointNumber], Nvec, Tvec);
          Obstacles[o]->force -= N.fn * Nvec + N.ft * Tvec;
          Obstacles[o]->Neighbors.push_back(N);
        }
      }
    } else if (token == "Scheduled") {
      std::string scheduledName;
      file >> scheduledName;
      Scheduler *sch = Factory<Scheduler>::Instance()->Create(scheduledName);
      if (sch != nullptr) {
        sch->plug(this);
        sch->read(file);
        Scheduled.push_back(sch);
      } else {
        Logger::warn("Scheduler {} is unknown!", scheduledName);
      }
    } else if (token == "Spy") {
      std::string spyName;
      file >> spyName;
      Spy *spy = Factory<Spy>::Instance()->Create(spyName);
      if (spy != nullptr) {
        spy->plug(this);
        spy->read(file);
        Spies.push_back(spy);
      } else {
        Logger::warn("Spy {} is unknown!", spyName);
      }
    } else if (token == "Nodes") {
      size_t nb;
      file >> nb;
      if (nodes.empty()) {
        Logger::critical("@MPMbox::read, 'Nodes' comes before the grid is defined");
        Logger::critical("  The grid has to be built first, with set_node_grid");
        exit(EXIT_FAILURE);
      }
      size_t in;
      for (size_t n = 0; n < nb; n++) {
        file >> in;
        if (in >= nodes.size()) {
          Logger::critical("@MPMbox::read, node number {} is outside the grid ({} nodes)", in, nodes.size());
          exit(EXIT_FAILURE);
        }
        file >> nodes[in].q >> nodes[in].f >> nodes[in].fb >> nodes[in].mass >> nodes[in].xfixed >> nodes[in].yfixed;
      }
    } else if (token == "Elem") {
      size_t nb;
      file >> element::nbNodes >> nb;
      if (element::nbNodes != 4 && element::nbNodes != 16) {
        Logger::critical("@MPMbox::read, 'Elem' declares {} nodes per element; only 4 and 16 are possible",
                         element::nbNodes);
        exit(EXIT_FAILURE);
      }
      Elem.clear();
      element E;
      for (size_t e = 0; e < nb; e++) {

        for (size_t r = 0; r < (size_t)element::nbNodes; r++) { file >> E.I[r]; }
        Elem.push_back(E);
      }
    } else if (token == "MPs") {
      size_t nb;
      file >> nb;
      MP.clear();
      MaterialPoint P;
      std::string modelName;
      for (size_t iMP = 0; iMP < nb; iMP++) {
        // FIXME
        // Il faut changer les sorties suivantes (enlever stressCorrection, ajouter hardeningForce, mettre
        // outOfPlaneStress à cote de stress)
        // Pas maintenant, pour ne pas casser la compatibilité...
        file >> modelName >> P.nb >> P.groupNb >> P.vol0 >> P.vol >> P.density >> P.pos >> P.vel >> P.strain >>
            P.plasticStrain >> P.stress >> P.stressCorrection >> P.splitCount >> P.F >> P.outOfPlaneStress >>
            P.contactf;

        auto itCM = models.find(modelName);
        if (itCM == models.end()) {
          Logger::critical("@MPMbox::read, Material Point {} refers to the model '{}', which is not defined", iMP,
                           modelName);
          exit(EXIT_FAILURE);
        }
        P.constitutiveModel = itCM->second;
        P.constitutiveModel->init(P);
        P.constitutiveModel->key = modelName;

        P.mass = P.vol * P.density;
        P.size = sqrt(P.vol0);
        MP.push_back(P);
      }
    } else if (token == "Nodes") {
      if (nodes.empty()) {
        Logger::warn("@MPMbox::read, cannot set the node-datasets if the grid has not been set (with a command)");
      }
      size_t nbNodes = 0;
      file >> nbNodes;
      if (nbNodes != nodes.size()) {
        Logger::warn("@MPMbox::read, The number of nodes is not compatible with the grid");
      }
      for (size_t in = 0; in < nodes.size(); in++) {
        file >> nodes[in].q >> nodes[in].f >> nodes[in].fb >> nodes[in].mass >> nodes[in].xfixed >> nodes[in].yfixed;
      }
    } else { // it is possible that the keyword corresponds to a command-pluggin
      Command *com = Factory<Command>::Instance()->Create(token);
      if (com != nullptr) {
        com->plug(this);
        com->read(file);
        com->exec();
      } else {
        Logger::warn("@MPMbox::read, what do you mean by '{}'?", token);
      }
    }

    file >> token;
  } // end while-loop

  // Some checks before running a simulation
  if (!shapeFunction) {
    std::string defaultShapeFunction = "Linear";
    shapeFunction                    = Factory<ShapeFunction>::Instance()->Create(defaultShapeFunction);
    Logger::info("No ShapeFunction defined, automatically set to 'Linear'");
  }

  if (!oneStep) {
    std::string defaultOneStep = "ModifiedLagrangian";
    oneStep                    = Factory<OneStep>::Instance()->Create(defaultOneStep);
    Logger::info("No OneStep type defined, automatically set to 'ModifiedLagrangian'");
  }
  dtInitial = dt;

  // If at least one Mp is double-scale, so hasDoubleScale is true
  CHCL.hasDoubleScale = false;
  for (size_t p = 0; p < MP.size(); p++) {
    if (MP[p].isDoubleScale == true) {
      CHCL.hasDoubleScale = true;
      break;
    }
  }
}

//
// Read a configuration file.
//
// Opens a file named 'conf<num>.txt' in the
// 'result_folder' directory, and reads in the configuration
// of the MPMbox object from that file. The configuration file is
// assumed to have been written by a call to the 'save'
// method.
//
void MPMbox::read(int num) {
  // Open file
  char name[256];
  snprintf(name, 256, "%s/conf%d.txt", result_folder.c_str(), num);
  Logger::info("Read {}", name);
  read(name);
}

//
// Write a configuration file.
//
// Writes a configuration file named 'conf<num>.txt' in the
// 'result_folder' directory, which contains all the
// information needed to reconstruct the current state of the MPMbox
// object. The configuration file is written in a format that can be
// read by a call to the 'read' method.
//
//
void MPMbox::save(const char *name) {
  std::ofstream file(name);

  file << "# MPM_CONFIGURATION_FILE Version May 2021\n";

  if (planeStrain == true) { file << "planeStrain\n"; }
  file << "oneStepType " << oneStep->getRegistrationName() << '\n';
  file << "result_folder " << result_folder << "\n";
  file << "tolmass " << tolmass << '\n';

  if (activePIC == true) {
    file << "enablePIC " << 1.0 - ratioFLIP << '\n';
  } else {
    file << "disablePIC" << '\n';
  }

  file << "gravity " << gravity << '\n';

  file << "CHCL.minDEMstep " << CHCL.minDEMstep << '\n';
  file << "CHCL.rateAverage " << CHCL.rateAverage << '\n';
  file << "CHCL.limitTimeStepFactor " << CHCL.limitTimeStepFactor << '\n';
  file << "CHCL.criticalDEMTimeStepFactor " << CHCL.criticalDEMTimeStepFactor << '\n';

  file << "finalTime " << finalTime << '\n';
  file << "proxPeriod " << proxPeriod << '\n';
  file << "confPeriod " << confPeriod << '\n';
  file << "dt " << dt << '\n';
  file << "t " << t << '\n';
  file << "splitting " << splitting << '\n';
  // Without these three, a restart of a computation with splitting silently
  // used the default values instead of the ones that were in force.
  file << "splitCriterionValue " << splitCriterionValue << '\n';
  file << "MaxSplitNumber " << MaxSplitNumber << '\n';
  file << "shearLimit " << shearLimit << '\n';
  file << "securDistFactor " << securDistFactor << '\n';
  file << "ShapeFunction " << shapeFunction->getRegistrationName() << '\n';

  for (size_t sc = 0; sc < Scheduled.size(); sc++) {
    file << "Scheduled ";
    Scheduled[sc]->write(file);
  }

  std::map<std::string, ConstitutiveModel *>::iterator itModel;
  for (itModel = models.begin(); itModel != models.end(); ++itModel) {
    file << "model " << itModel->second->getRegistrationName() << ' ' << itModel->first << ' ';
    itModel->second->write(file);
  }

  // MP-Obstacle interaction properties
  size_t ngroup = dataTable.get_ngroup();
  for (size_t MPgroup = 0; MPgroup < ngroup; MPgroup++) {
    for (size_t ObstGroup = 0; ObstGroup < ngroup; ObstGroup++) {
      if (dataTable.isDefined(id_kn, MPgroup, ObstGroup)) {
        file << "set kn " << MPgroup << ' ' << ObstGroup << ' ' << dataTable.get(id_kn, MPgroup, ObstGroup) << '\n';
      }
      if (dataTable.isDefined(id_kt, MPgroup, ObstGroup)) {
        file << "set kt " << MPgroup << ' ' << ObstGroup << ' ' << dataTable.get(id_kt, MPgroup, ObstGroup) << '\n';
      }
      if (dataTable.isDefined(id_mu, MPgroup, ObstGroup)) {
        file << "set mu " << MPgroup << ' ' << ObstGroup << ' ' << dataTable.get(id_mu, MPgroup, ObstGroup) << '\n';
      }
      if (dataTable.isDefined(id_en2, MPgroup, ObstGroup)) {
        file << "set en2 " << MPgroup << ' ' << ObstGroup << ' ' << dataTable.get(id_en2, MPgroup, ObstGroup) << '\n';
      }
      if (dataTable.isDefined(id_viscRate, MPgroup, ObstGroup)) {
        file << "set viscRate " << MPgroup << ' ' << ObstGroup << ' ' << dataTable.get(id_viscRate, MPgroup, ObstGroup)
             << '\n';
      }
      if (dataTable.isDefined(id_dn0, MPgroup, ObstGroup)) {
        file << "set dn0 " << MPgroup << ' ' << ObstGroup << ' ' << dataTable.get(id_dn0, MPgroup, ObstGroup) << '\n';
      }
      if (dataTable.isDefined(id_dt0, MPgroup, ObstGroup)) {
        file << "set dt0 " << MPgroup << ' ' << ObstGroup << ' ' << dataTable.get(id_dt0, MPgroup, ObstGroup) << '\n';
      }
    }
  }

  // fixe-grid
  file << "set_node_grid Nx.Ny.lx.ly " << Grid.Nx << ' ' << Grid.Ny << ' ' << Grid.lx << ' ' << Grid.ly << '\n';
  // This is a command that will set the Elements and the nodes also

  // The node datasets (not all)
  std::vector<size_t> savedNodes;
  for (size_t in = 0; in < nodes.size(); in++) {
    if (nodes[in].q.x == 0.0 && nodes[in].q.y == 0.0 && nodes[in].f.x == 0.0 && nodes[in].f.y == 0.0 &&
        nodes[in].fb.x == 0.0 && nodes[in].fb.y == 0.0 && nodes[in].mass == 0.0 && nodes[in].xfixed == 0 &&
        nodes[in].yfixed == 0) {
      continue;
    }
    savedNodes.push_back(in);
  }
  file << "Nodes " << savedNodes.size() << '\n';
  for (size_t n = 0; n < savedNodes.size(); n++) {
    size_t in = savedNodes[n];
    file << in << ' ' << nodes[in].q << ' ' << nodes[in].f << ' ' << nodes[in].fb << ' ' << nodes[in].mass << ' '
         << nodes[in].xfixed << ' ' << nodes[in].yfixed << '\n';
  }

  // Obstacles
  for (size_t iObst = 0; iObst < Obstacles.size(); iObst++) {
    file << "Obstacle " << Obstacles[iObst]->getRegistrationName() << ' ';
    Obstacles[iObst]->write(file);
  }

  // Boudary Force Laws
  for (size_t i = 0; i < BFLCommandStored.size(); i++) { file << BFLCommandStored[i] << '\n'; }

  // Material points
  file << "MPs " << MP.size() << '\n';
  file << std::scientific << std::setprecision(std::numeric_limits<double>::digits10 + 1);
  for (size_t iMP = 0; iMP < MP.size(); iMP++) {
    file << MP[iMP].constitutiveModel->key << ' ' << MP[iMP].nb << ' ' << MP[iMP].groupNb << ' ' << MP[iMP].vol0 << ' '
         << MP[iMP].vol << ' ' << MP[iMP].density << ' ' << MP[iMP].pos << ' ' << MP[iMP].vel << ' ' << MP[iMP].strain
         << ' ' << MP[iMP].plasticStrain << ' ' << MP[iMP].stress << ' ' << MP[iMP].stressCorrection << ' '
         << MP[iMP].splitCount << ' ' << MP[iMP].F << ' ' << MP[iMP].outOfPlaneStress << ' ' << MP[iMP].contactf
         << '\n';
  }

  // Obstacle Neighbors
  if (!Obstacles.empty()) {
    file << "ObstacleNeighbors\n";
    for (size_t iObst = 0; iObst < Obstacles.size(); iObst++) {
      file << Obstacles[iObst]->Neighbors.size() << '\n';
      for (size_t n = 0; n < Obstacles[iObst]->Neighbors.size(); n++) {
        file << Obstacles[iObst]->Neighbors[n].PointNumber << ' ' << Obstacles[iObst]->Neighbors[n].fn << ' '
             << Obstacles[iObst]->Neighbors[n].dn << ' ' << Obstacles[iObst]->Neighbors[n].ft << ' '
             << Obstacles[iObst]->Neighbors[n].dt << ' ' << Obstacles[iObst]->Neighbors[n].sigma_n << '\n';
      }
    }
  }
}

//
// Saves the current state of the simulation to a file
//
// The function saves the current state of the simulation to a file
// with the name 'result_folder/conf\*num*.txt'.
//
// The state of the simulation consists of the following information:
//   - the nodes of the Eulerian grid
//   - the Material Points (their position, velocity, strain, stress, etc.)
//   - the rigid obstacles
//   - the models used for the Material Points
//
// The function also logs the time and the number of Material Points
// of the simulation.
//
void MPMbox::save(int num) {
  char name[256];
  snprintf(name, 256, "%s/conf%d.txt", result_folder.c_str(), num);
  Logger::info("Save {}, #MP: {}, Time: {:.6f} ({:.1f}%)", name, MP.size(), t, 100.0 * t / finalTime);
  save(name);
}

//
// Initializes the MPMbox by setting up necessary directories and
// updating the previous positions of Material Points.
//
// This function performs the following actions:
//   - Creates the result folder if it doesn't exist.
//   - Creates individual folders for each tracked Material Point (MP)
//     for simulations with double scale.
//   - Updates the previous position of each Material Point to the current
//     position for future reference.
//
void MPMbox::init() {
  // If the result folder does not exist, it is created
  if (result_folder != "" && result_folder != "." && result_folder != "./") fileTool::create_folder(result_folder);

  // create folders for the tracked MP (double scale simulations)
  for (size_t iMP = 0; iMP < MP.size(); iMP++) {
    if (MP[iMP].isTracked == true) {
      char fname[256];
      snprintf(fname, 256, "%s/DEM_MP%zu", result_folder.c_str(), iMP);
      fileTool::create_folder(fname);
    }
  }

  for (size_t p = 0; p < MP.size(); p++) { MP[p].prev_pos = MP[p].pos; }
}

//
// Run the simulation.
//
// This function runs the simulation until the final time is reached.
// It will:
//   - Check if the Material Points are inside the grid area
//   - Perform the simulation steps
//   - Check for convergence requirements
//   - Save the configuration of the simulation at regular intervals
//   - Check for proximity between Material Points
//   - Split Material Points if necessary
//   - Execute/Record the spies at regular intervals
//   - Shutdown the spies at the end of the simulation
//
void MPMbox::run() {
  START_TIMER("run");

  // Check the settings that would make the time loop misbehave
  checkSettings();

  // Check wether the MPs stand inside the grid area
  MPinGridCheck();

  step = 0;

  while (t <= finalTime) {

    // checking convergence requirements
    convergenceConditions();

    if (step % confPeriod == 0) {
      save(iconf);

      // save DEM_MP conf-files
      if (CHCL.hasDoubleScale == true) {
        for (size_t p = 0; p < MP.size(); p++) {
          if (MP[p].isTracked) {
            char fname[256];
            snprintf(fname, 256, "%s/DEM_MP%zu/conf%i", result_folder.c_str(), p, iconf);
            MP[p].PBC->iconf = iconf;
            MP[p].PBC->t     = t;
            MP[p].PBC->tmax  = t;
            MP[p].PBC->saveConf(fname);
          }
        }
      }

      iconf++;
    }

    // The neighbor lists are rebuilt every proxPeriod steps, and also as soon as
    // the number of Material Points has changed -- a split shifts the indices
    // the lists are made of. number_MP_before_any_split is refreshed at the top
    // of advanceOneStep, i.e. BEFORE adaptativeRefinement: a split is therefore
    // seen at the next step.
    // A removal, on the other hand, happens just below, between this test and
    // advanceOneStep, so the refresh wipes it out and this guard never sees it.
    // RemoveMaterialPoint rebuilds the lists itself for that reason (see A7).
    if (step % proxPeriod == 0 || MP.size() != number_MP_before_any_split) {
      checkProximity();
    }

    for (size_t s = 0; s < Scheduled.size(); ++s) { Scheduled[s]->check(); }

    // run a step!
    int ret = oneStep->advanceOneStep(*this);
    if (ret == 1) break; // returns 1 only in trajectory analyses when contact is lost and normal vel is 1

    // Split MPs
    if (splitting) adaptativeRefinement();

    // Execute/Record the spies
    for (size_t s = 0; s < Spies.size(); ++s) {
      if ((step % Spies[s]->nstep) == 0) Spies[s]->exec();
      if ((step % Spies[s]->nrec) == 0) Spies[s]->record();
    }

    t += dt;
    step++;
  }

  // shutdown the spies
  for (size_t s = 0; s < Spies.size(); ++s) {
    Spies[s]->end(); // there is often nothing implemented
  }
}

//
// Check the proximity of the MPs and obstacles. This function is called every proxPeriod steps.
// It computes the security distance for each MP and obstacle as securDistFactor * velocity * dt * proxPeriod.
// It then calls the checkProximity function on each obstacle.
// see: MPMbox::proxPeriod, Obstacle::checkProximity
//
void MPMbox::checkProximity() {
  START_TIMER("checkProximity");
  // Compute securDist of MPs
  for (size_t p = 0; p < MP.size(); p++) { MP[p].securDist = securDistFactor * norm(MP[p].vel) * dt * proxPeriod; }

  for (size_t o = 0; o < Obstacles.size(); o++) {
    Obstacles[o]->securDist = securDistFactor * norm(Obstacles[o]->vel) * dt * proxPeriod;
    Obstacles[o]->checkProximity(*this);
  }
}

//
// Check the settings that the time loop cannot cope with.
//
// It is called by run(), and not by read(), so that the viewer -- which reads
// conf-files but never runs anything -- is never stopped by these checks.
//
// Two families:
//   - the periods used as a modulo. A zero period is an integer division by
//     zero, whose behaviour depends on the processor: SIGFPE on x86-64, but a
//     silent zero on AArch64, which makes the condition always true (a
//     conf-file written at every single step).
//   - the interaction parameters. DataTable::get does no bound checking at all,
//     and the number of groups only grows through the 'set' keyword: a group
//     that never appears in a 'set' line reads outside the vectors.
//
void MPMbox::checkSettings() {
  if (confPeriod <= 0) {
    Logger::critical("@MPMbox::checkSettings, confPeriod = {}; it is used as a modulo and has to be at least 1",
                     confPeriod);
    exit(EXIT_FAILURE);
  }
  if (proxPeriod <= 0) {
    Logger::critical("@MPMbox::checkSettings, proxPeriod = {}; it is used as a modulo and has to be at least 1",
                     proxPeriod);
    exit(EXIT_FAILURE);
  }
  for (size_t s = 0; s < Spies.size(); s++) {
    if (Spies[s]->nstep <= 0 || Spies[s]->nrec <= 0) {
      Logger::critical("@MPMbox::checkSettings, Spy #{} has nstep = {} and nrec = {}; both are used as a modulo "
                       "and have to be at least 1",
                       s, Spies[s]->nstep, Spies[s]->nrec);
      exit(EXIT_FAILURE);
    }
  }

  if (Obstacles.empty() || MP.empty()) { return; }

  std::set<int> groupsMP;
  std::set<int> groupsObs;
  for (size_t p = 0; p < MP.size(); p++) { groupsMP.insert(MP[p].groupNb); }
  for (size_t o = 0; o < Obstacles.size(); o++) { groupsObs.insert(Obstacles[o]->group); }

  const size_t ngroup = dataTable.get_ngroup();
  for (std::set<int>::iterator g1 = groupsMP.begin(); g1 != groupsMP.end(); ++g1) {
    for (std::set<int>::iterator g2 = groupsObs.begin(); g2 != groupsObs.end(); ++g2) {
      if (*g1 < 0 || *g2 < 0 || (size_t)(*g1) >= ngroup || (size_t)(*g2) >= ngroup) {
        Logger::critical("@MPMbox::checkSettings, no interaction parameter has ever been set for the pair "
                         "(MP group {}, obstacle group {})",
                         *g1, *g2);
        Logger::critical("  The table holds {} group(s). Add the missing 'set' lines, e.g. 'set kn {} {} 1e6'",
                         ngroup, *g1, *g2);
        exit(EXIT_FAILURE);
      }
      // The pair is inside the table, so reading it is safe; a parameter left
      // undefined is zero, which is legitimate for some laws (viscRate) but
      // almost never for kn.
      if (!dataTable.isDefined(id_kn, *g1, *g2)) {
        Logger::warn("@MPMbox::checkSettings, 'set kn {} {} ...' is missing; the normal stiffness between MP group "
                     "{} and obstacle group {} is zero, so the contact will not push back",
                     *g1, *g2, *g1, *g2);
      }
      if (!dataTable.isDefined(id_mu, *g1, *g2)) {
        Logger::warn("@MPMbox::checkSettings, 'set mu {} {} ...' is missing; the friction between MP group {} and "
                     "obstacle group {} is zero",
                     *g1, *g2, *g1, *g2);
      }
    }
  }
}

//
// Check if any Material Point is outside the grid before the start of the simulation.
//
// This function will check if any Material Point is outside the grid before the start of the simulation. If any
// Material Point is found to be outside the grid, a warning message will be printed.
//
//
// Build the nodes and the elements from Grid.Nx, Grid.Ny, Grid.lx and Grid.ly,
// which the calling command has already set.
//
// The shape function decides how many nodes an element holds, and therefore how
// many ghost rings the node grid needs: a 16-node element reads the ring around
// itself, which does not exist on the border of an unpadded grid. That was the
// defect A3 -- twelve of the sixteen indices were left at zero there, and every
// shape function of a border element was silently piled onto the node 0.
//
// set_node_grid and new_set_grid used to hold two verbatim copies of this code,
// including the same border special case.
//
void MPMbox::buildGrid() {
  Grid.pad = (element::nbNodes == 16) ? 1 : 0;
  const long pad = (long)Grid.pad;

  nodes.clear();
  nodes.resize(Grid.nbNodes());
  for (long j = -pad; j <= (long)Grid.Ny + pad; j++) {
    for (long i = -pad; i <= (long)Grid.Nx + pad; i++) {
      const size_t n = Grid.nodeNumber(i, j);
      nodes[n].number = n;
      nodes[n].pos.set((double)i * Grid.lx, (double)j * Grid.ly);
    }
  }

  Elem.clear();
  Elem.reserve(Grid.Nx * Grid.Ny);
  element E;
  for (long j = 0; j < (long)Grid.Ny; j++) {
    for (long i = 0; i < (long)Grid.Nx; i++) {
      for (size_t r = 0; r < element::nbNodes; r++) {
        E.I[r] = Grid.nodeNumber(i + element::dxOff[r], j + element::dyOff[r]);
      }
      Elem.push_back(E);
    }
  }

  liveNodeNum.clear();
  liveNodeNum.reserve(nodes.size());
  for (size_t n = 0; n < nodes.size(); n++) { liveNodeNum.push_back(n); }

  if (Grid.pad > 0) {
    Logger::info("@MPMbox::buildGrid, {} x {} elements of {} x {}, {} x {} nodes including {} ghost ring", Grid.Nx,
                 Grid.Ny, Grid.lx, Grid.ly, Grid.nbNodeCols(), Grid.nbNodeRows(), Grid.pad);
  }
}

void MPMbox::MPinGridCheck() {
  // checking for MP outside the grid before the start of the simulation
  // The bounds are the same as in ShapeFunction::locateElement (the far sides
  // are excluded), so that this warning agrees with the check that will stop
  // the computation at the first step.
  for (size_t p = 0; p < MP.size(); p++) {
    if (MP[p].pos.x >= (double)Grid.Nx * Grid.lx || MP[p].pos.x < 0.0 ||
        MP[p].pos.y >= (double)Grid.Ny * Grid.ly || MP[p].pos.y < 0.0) {
      Logger::warn("@MPMbox::MPinGridCheck, Check before simulation: MP position (x={}, y={}) is not inside the grid",
                   MP[p].pos.x, MP[p].pos.y);
    }
  }
}

//
// Check for convergence conditions.
//
// This function checks for several convergence conditions:
//   - Passthrough velocity condition: the timestep should be smaller than the smallest radius
//     of the MPs divided by the maximum velocity of the MPs.
//   - Collision condition: the timestep should be smaller than the minimum mass of the MPs
//     divided by the maximum normal stiffness of the obstacles.
//   - CFL condition: the timestep should be smaller than the smallest rayon of the MPs divided
//     by the maximum speed of sound of the MPs. If any of these conditions is not satisfied,
//     the timestep is adjusted to half the critical value.
//
// see: MPMbox::dt, MPMbox::dtInitial
//
void MPMbox::convergenceConditions() {
  START_TIMER("convergenceConditions");

  // finding necessary parameters
  double inf        = std::numeric_limits<double>::max();
  double YoungMax   = -inf;
  double PoissonMax = -inf;
  double rhoMin     = inf;
  double rayMin     = inf;
  double knMax      = -inf;
  double massMin    = inf;
  double velMax     = -inf;
  std::set<int> groupsMP;
  std::set<int> groupsObs;
  for (size_t p = 0; p < MP.size(); ++p) {
    YoungMax   = std::max(MP[p].constitutiveModel->getYoung(), YoungMax);
    PoissonMax = std::max(MP[p].constitutiveModel->getPoisson(), PoissonMax);
    rhoMin     = std::min(MP[p].density, rhoMin);
    massMin    = std::min(MP[p].mass, massMin);
    velMax     = std::max(MP[p].vel * MP[p].vel, velMax);
    rayMin     = std::min(MP[p].vol * Mth::invPi, rayMin);
    groupsMP.insert((size_t)(MP[p].groupNb));
  }
  velMax = sqrt(velMax);
  rayMin = sqrt(rayMin);

  if (Obstacles.size() > 0) {
    for (size_t o = 0; o < Obstacles.size(); ++o) { groupsObs.insert(Obstacles[o]->group); }
  }

  std::set<int>::iterator it;
  std::set<int>::iterator it2;
  for (it = groupsMP.begin(); it != groupsMP.end(); ++it) {
    for (it2 = groupsObs.begin(); it2 != groupsObs.end(); ++it2) {
      if (dataTable.get(id_kn, *it, *it2) > knMax) { knMax = dataTable.get(id_kn, *it, *it2); }
    }
  }

  // Collect the criteria that can actually be evaluated. Each of them is an
  // UPPER bound on the time step, so the binding one is the SMALLEST -- which
  // is what the comment below has always said, while the code took the largest.
  //
  // A criterion is left out rather than replaced by a fallback when the data it
  // needs is missing. Taking sqrt(massMin / knMax) with no obstacle at all used
  // to give sqrt of a negative number, since knMax was still at -DBL_MAX: the
  // resulting NaN then propagated into the comparison, whose outcome is not
  // specified.
  std::vector<double> crits;
  std::vector<std::string> names;

  // Passthrough: a Material Point must not jump over its own size in one step.
  if (velMax > 1e-6) {
    crits.push_back(rayMin / velMax);
    names.push_back("passthrough velocity");
  }

  // Collision: period of the oscillator made of a MP and the contact stiffness.
  if (knMax > 0.0) {
    crits.push_back(sqrt(massMin / knMax));
    names.push_back("collision");
  }

  // CFL: the elastic wave must not cross a Material Point in one step. CHCL_DEM
  // returns -1 by convention for both moduli, and a Poisson ratio of 0.5 makes
  // the bulk modulus infinite.
  if (YoungMax > 0.0 && PoissonMax >= 0.0 && PoissonMax < 0.5) {
    double Kmax = YoungMax / (1.0 - 2.0 * PoissonMax);
    crits.push_back(rayMin / sqrt(Kmax / rhoMin));
    names.push_back("CFL");
  }

  if (crits.empty()) { return; }

  size_t iworst = 0;
  for (size_t i = 1; i < crits.size(); i++) {
    if (crits[i] < crits[iworst]) { iworst = i; }
  }
  const double criticalDt = crits[iworst];

  if (step == 0) {
    Logger::debug("Current dt: {}", dt);
    for (size_t i = 0; i < crits.size(); i++) {
      Logger::debug("dt_crit/dt ({}): {:.3f}", names[i], crits[i] / dt);
    }
  }

  // A 1 % margin, without which the adjustment fires again at the very next
  // step: dt has just been set to 0.5 * criticalDt, and criticalDt moves in its
  // last digits from one step to the next (the volume of the Material Points
  // changes). Without the margin, a single run logs hundreds of thousands of
  // adjustments of one part in 1e12.
  const double dtMax = 0.5 * criticalDt;
  if (dt > 1.01 * dtMax) {
    Logger::info("@MPMbox::convergenceConditions, timestep {} is too large for the '{}' criterion (step {})", dt,
                 names[iworst], step);
    dt        = dtMax;
    dtInitial = dt;
    Logger::info("--> Adjusting time step to {}", dt);
    for (size_t i = 0; i < crits.size(); i++) {
      Logger::debug("dt_crit/dt ({}): {:.3f}", names[i], crits[i] / dt);
    }
  }
}

// ===================================================
//  Functions called by the OneStep-derived functions
// ===================================================

//
// Make room in the side arrays -- shape functions, model state, previous
// deformation gradient -- for the current number of Material Points.
//
// Called at the beginning of each time step and of postProcess. The test costs
// nothing, and the arrays only grow when points are created -- by a command, by
// the adaptive splitting -- or when the shape function changes the number of
// nodes per element. The values themselves are recomputed at every step by
// computeInterpolationValues, so there is nothing to preserve.
//
void MPMbox::resizeMPArrays() {
  const size_t need = MP.size() * element::nbNodes;
  if (shapeN.size() != need) {
    shapeN.resize(need, 0.0);
    shapeGradN.resize(need);
  }
  if (modelStateStore.size() != MP.size()) { modelStateStore.resize(MP.size()); }
  if (CHCL.hasDoubleScale == true && prevFstore.size() != MP.size()) {
    prevFstore.resize(MP.size(), mat4r::unit());
  }
}

//
// Rebuild liveNodeNum, the list of the nodes that carry at least one Material
// Point at this time step.
//
// The nodes are marked in a scratch array indexed by node number, with the
// rebuild counter as the mark: a node already seen during this rebuild carries
// the current mark and is not added twice. No search, no sorting, no
// allocation once the array is dimensioned.
//
// The previous implementation inserted every (Material Point, node) pair into a
// std::set -- 6144 insertions into a red-black tree per step on the reference
// benchmark, each one allocating. It weighed 13 % of the time step, against
// 2 % now. Replacing the set by push_back + sort + unique, which was tried,
// brings nothing: the sorting costs as much as the tree.
//
// The list is NOT sorted any more. Nothing depends on it: the nodes are then
// treated one by one, with no summation across the list, and the order stays
// deterministic since it follows the order of the Material Points.
//
void MPMbox::updateLiveNodeList() {
  START_TIMER("live node list");

  if (nodeStamp.size() != nodes.size()) {
    nodeStamp.assign(nodes.size(), 0);
    stampTag = 0;
  }
  ++stampTag;
  if (stampTag == 0) { // wrapped around after 4e9 rebuilds
    std::fill(nodeStamp.begin(), nodeStamp.end(), 0);
    stampTag = 1;
  }

  liveNodeNum.clear();
  for (size_t p = 0; p < MP.size(); p++) {
    size_t *I = &(Elem[MP[p].e].I[0]);
    for (size_t r = 0; r < element::nbNodes; r++) {
      if (nodeStamp[I[r]] != stampTag) {
        nodeStamp[I[r]] = stampTag;
        liveNodeNum.push_back(I[r]);
      }
    }
  }
}

//
// Update the velocity gradient for all material points.
//
// The velocity gradient of a material point is computed as the sum of the
// product of the gradient of the shape function and the velocity of the
// corresponding node. The velocity gradient is stored in the velGrad member
// variable of each material point.
//
// This function is called by the OneStep functions.
//
void MPMbox::updateVelocityGradient() {
  START_TIMER("updateVelocityGradient");

  // The reset belongs here, and not in the integration schemes: the loop below
  // accumulates, so a scheme that forgets to clear velGrad sums the gradients
  // of every step since the beginning. UpdateStressFirst and UpdateStressLast
  // did exactly that -- their comment announced the reset, the code did not do
  // it -- and their deformation gradient diverged in a few thousand steps.
  for (size_t p = 0; p < MP.size(); p++) { MP[p].velGrad.reset(); }

  size_t *I;
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    const vec2r *gNp = gradN(p);
    for (size_t r = 0; r < element::nbNodes; r++) {
      MP[p].velGrad.xx += (gNp[r].x * nodes[I[r]].vel.x);
      MP[p].velGrad.yy += (gNp[r].y * nodes[I[r]].vel.y);
      MP[p].velGrad.xy += (gNp[r].y * nodes[I[r]].vel.x);
      MP[p].velGrad.yx += (gNp[r].x * nodes[I[r]].vel.y);
    }
  }
}

//
// Limit the time-step for DEM simulations.
//
// If CHCL.limitTimeStepFactor is positive, this function limits the time-step
// dt to a value that is a fraction of the critical time-step for DEM simulations.
// The critical time-step is computed as CHCL.limitTimeStepFactor * Rmin / maxi,
// where Rmin is the minimum radius of the MPs and maxi is the maximum absolute
// value of the components of the velocity gradient tensor of the MPs.
//
// see: MPMbox::dt, MPMbox::dtInitial, MPMbox::CHCL
//
void MPMbox::limitTimeStepForDEM() {
  START_TIMER("limitTimeStepForDEM");
  if (CHCL.limitTimeStepFactor <= 0.0) return;

  dt           = dtInitial;
  double dtmax = 0.0;

  for (size_t p = 0; p < MP.size(); p++) {
    if (MP[p].isDoubleScale) {
      mat9r VG3D;
      VG3D.xx = MP[p].velGrad.xx;
      VG3D.xy = MP[p].velGrad.xy;
      VG3D.yx = MP[p].velGrad.yx;
      VG3D.yy = MP[p].velGrad.yy;
      // VG3D.zz = 0.0;  // assuming plane strain
      VG3D = VG3D * MP[p].PBC->Cell.h;
      // clang-format off
      double maxi = std::max({fabs(VG3D.xx), fabs(VG3D.xy), fabs(VG3D.xz),
				                      fabs(VG3D.yx), fabs(VG3D.yy), fabs(VG3D.yz),
                              fabs(VG3D.zx), fabs(VG3D.zy), fabs(VG3D.zz)});
      // clang-format on

      if (maxi < 1e-12) dtmax = dt;
      else dtmax = CHCL.limitTimeStepFactor * MP[p].PBC->Rmin / maxi;

      dt = (dtmax <= dt) ? dtmax : dt;
    }
  }

  Logger::trace("MPM time-step dt = {} at the end limitTimeStepForDEM", dt);
}

//
// Updates the transformation gradient F for all material points.
//
// The transformation gradient F is computed at each time-step as F = (I + dt * L) * F,
// where I is the identity matrix, dt is the time-step, L is the velocity gradient
// tensor computed by updateVelocityGradient, and F is the transformation gradient
// at the previous time-step.
//
// This function is called by the OneStep functions.
//
void MPMbox::updateTransformationGradient() {
  START_TIMER("updateTransformationGradient");
  updateVelocityGradient();
  if (CHCL.hasDoubleScale == true) limitTimeStepForDEM();

  // prev_F n'est lu que par CHCL_DEM : 32 octets ecrits par point et par pas,
  // pour rien, dans un calcul simple echelle.
  if (CHCL.hasDoubleScale == true) {
    for (size_t p = 0; p < MP.size(); p++) {
      prevF(p) = MP[p].F;
      MP[p].F  = (mat4r::unit() + dt * MP[p].velGrad) * MP[p].F;
    }
  } else {
    for (size_t p = 0; p < MP.size(); p++) { MP[p].F = (mat4r::unit() + dt * MP[p].velGrad) * MP[p].F; }
  }
}

//
// Perform adaptive refinement of material points.
//
// This function checks each material point for excessive shearing or deformation
// and performs splitting if necessary. The transformation gradient F is set to
// identity when shearing exceeds a predefined limit. If the deformation satisfies
// certain criteria, the material point is split into two, with properties adjusted
// accordingly. The split is either along the x or y direction, based on the
// deformation extent. This function is part of a refinement strategy to enhance
// simulation accuracy by adapting the discretization dynamically.
//
// - If the shearing in F is too large, F is reset to the identity matrix.
// - Splitting occurs if the deformation extent in one direction exceeds a critical value.
// - Splits are performed along the axis with the larger deformation extent.
// - The function also checks for extreme shearing conditions after the splitting criteria.
//
void MPMbox::adaptativeRefinement() {
  START_TIMER("adaptativeRefinement");

  // The number of Material Points grows inside the loop. The bound is fixed
  // beforehand so that a point created here is examined at the NEXT call and
  // not in the same pass: it used to be split again straight away, as many
  // times as MaxSplitNumber allowed, and the outcome depended on the order in
  // which the points happened to be stored.
  const size_t nbBefore = MP.size();

  // Next free identifier. MP2 used to inherit the number of the point it comes
  // from, so the numbers were no longer unique.
  size_t nextNumber = 0;
  for (size_t p = 0; p < MP.size(); p++) { nextNumber = std::max(nextNumber, MP[p].nb + 1); }

  for (size_t p = 0; p < nbBefore; p++) {
    if (MP[p].splitCount > MaxSplitNumber) continue;

    // Splitting a Material Point that carries a DEM cell would leave the two
    // halves sharing the same PBC3Dbox: the same cell would be deformed twice
    // per step, and by two threads at once in the OpenMP loop of
    // ModifiedLagrangian.
    if (MP[p].isDoubleScale == true) {
      static bool warned = false;
      if (!warned) {
        Logger::warn("@MPMbox::adaptativeRefinement, double-scale Material Points are not split "
                     "(their DEM cell cannot be duplicated)");
        warned = true;
      }
      continue;
    }

    // setting F to identity if shearing is too large
    // (shearLimit < 0 disables the mechanism, which is the default)
    if (shearLimit > 0.0 && (fabs(MP[p].F.xy) > shearLimit or fabs(MP[p].F.yx) > shearLimit)) {
      MP[p].F.xx = 1;
      MP[p].F.xy = 0;
      MP[p].F.yx = 0;
      MP[p].F.yy = 1;
    }

    double XSquaredExtent = (MP[p].F.xx * MP[p].F.xx + MP[p].F.yx * MP[p].F.yx);
    double YSquaredExtent = (MP[p].F.xy * MP[p].F.xy + MP[p].F.yy * MP[p].F.yy);
    double SquaredCrit    = splitCriterionValue * splitCriterionValue;

    bool critX = ((XSquaredExtent / YSquaredExtent) >= SquaredCrit);
    bool critY = ((YSquaredExtent / XSquaredExtent) >= SquaredCrit);

    if ((critX || critY) == true) {
      MP[p].splitCount += 1;
      double halfSizeMP = 0.5 * MP[p].size;

      MP[p].mass *= 0.5;
      MP[p].vol *= 0.5;

      // All properties are copied thank to the auto-generated copy-ctor
      MaterialPoint MP2 = MP[p];
      MP2.nb            = nextNumber++;
      // MP[p] will go to the left or bottom
      // and MP2 will go to the right or top
      //
      // Remark: vol0 and size are deliberately left untouched. The reference
      // footprint stays the same and it is F that carries the division, so the
      // current area |det F| * size^2 is indeed halved -- which is also what
      // makes 'vol = F.det() * vol0' hold in UpdateStressFirst/Last, and what
      // makes 'size = sqrt(vol0)' still correct when the conf-file is read back.

      if (critX == true) { // -> left-right splitting
        vec2r sx = MP[p].F * vec2r(halfSizeMP, 0.0);
        MP[p].pos -= 0.5 * sx;
        MP2.pos += 0.5 * sx;

        MP[p].F.xx *= 0.5;
        MP[p].F.yx *= 0.5;

        MP2.F.xx = MP[p].F.xx;
        MP2.F.yx = MP[p].F.yx;

        MP.push_back(MP2);
      } else { // -> top-bottom splitting
        vec2r sy = MP[p].F * vec2r(0.0, halfSizeMP);
        MP[p].pos -= 0.5 * sy;
        MP2.pos += 0.5 * sy;

        MP[p].F.xy *= 0.5;
        MP[p].F.yy *= 0.5;

        MP2.F.xy = MP[p].F.xy;
        MP2.F.yy = MP[p].F.yy;

        MP.push_back(MP2);
      }
    } // end if if ((critX || critY) == true)

    // checking extremeShearing after checking the above criteria
    if (extremeShearing) {
      bool critExtremeShearing =
          (MP[p].F.xx / MP[p].F.xy < extremeShearingval || MP[p].F.yy / MP[p].F.yx < extremeShearingval);
      if (critExtremeShearing) {
        // ... ???
      }
    }

  } // end for loop over MPs
}

//
// Post-process the MPs after time stepping.
//
// Compute the smoothed data (vel, stress, velGrad, outOfPlaneStress, pos, strain,
// rho) from the MPs by using the shape functions. The data is stored in the
// Data vector.
//
// Data is the vector of ProcessedDataMP where the smoothed data will be stored.
//
void MPMbox::postProcess(std::vector<ProcessedDataMP> &Data) {
  Data.clear();
  Data.resize(MP.size());

  // Preparation for smoothed data
  size_t *I;
  resizeMPArrays();
  for (size_t p = 0; p < MP.size(); p++) { shapeFunction->computeInterpolationValues(*this, p); }

  // Update Vector of node indices
  updateLiveNodeList();

  // Reset nodal mass
  for (size_t n = 0; n < liveNodeNum.size(); n++) {
    nodes[liveNodeNum[n]].mass             = 0.0;
    nodes[liveNodeNum[n]].outOfPlaneStress = 0.0;
    nodes[liveNodeNum[n]].vel.reset();
    nodes[liveNodeNum[n]].stress.reset();
  }

  // Nodal mass
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    const double *Np = N(p);
    for (size_t r = 0; r < element::nbNodes; r++) {
      nodes[I[r]].mass += Np[r] * MP[p].mass;
      nodes[I[r]].outOfPlaneStress += Np[r] * MP[p].outOfPlaneStress;
    }
  }

  // smooth procedure
  // MP -> nodes
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    const double *Np = N(p);
    for (size_t r = 0; r < element::nbNodes; r++) {
      nodes[I[r]].vel += Np[r] * MP[p].mass * MP[p].vel / nodes[I[r]].mass;
      nodes[I[r]].stress += Np[r] * MP[p].mass * MP[p].stress / nodes[I[r]].mass;
    }
  }
  // nodes -> MPs
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    const double *Np = N(p);
    const vec2r *gNp = gradN(p);
    for (size_t r = 0; r < element::nbNodes; r++) {
      Data[p].vel += nodes[I[r]].vel * Np[r];
      Data[p].stress += nodes[I[r]].stress * Np[r];
      Data[p].velGrad.xx += (gNp[r].x * nodes[I[r]].vel.x);
      Data[p].velGrad.yy += (gNp[r].y * nodes[I[r]].vel.y);
      Data[p].velGrad.xy += (gNp[r].y * nodes[I[r]].vel.x);
      Data[p].velGrad.yx += (gNp[r].x * nodes[I[r]].vel.y);
      Data[p].outOfPlaneStress += Np[r] * nodes[I[r]].outOfPlaneStress;
    }
    Data[p].pos    = MP[p].pos;
    Data[p].strain = MP[p].F;
    Data[p].rho    = MP[p].density;
  }

  // corners from F (supposed to be already computed)
  for (size_t p = 0; p < MP.size(); p++) {
    double halfSizeMP = 0.5 * MP[p].size;

    Data[p].corner[0] = MP[p].pos + MP[p].F * vec2r(-halfSizeMP, -halfSizeMP);
    Data[p].corner[1] = MP[p].pos + MP[p].F * vec2r(halfSizeMP, -halfSizeMP);
    Data[p].corner[2] = MP[p].pos + MP[p].F * vec2r(halfSizeMP, halfSizeMP);
    Data[p].corner[3] = MP[p].pos + MP[p].F * vec2r(-halfSizeMP, halfSizeMP);
  }
}
