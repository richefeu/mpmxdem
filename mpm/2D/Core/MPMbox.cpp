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

#include "Spies/EnergyBalance.hpp"
#include "Spies/MPTracking.hpp"
#include "Spies/MeanStress.hpp"
#include "Spies/ObstacleTracking.hpp"
#include "Spies/Spy.hpp"
#include "Spies/Work.hpp"
#include "Spies/ElasticBeamDev.hpp"

#include "Schedulers/GravityRamp.hpp"
#include "Schedulers/PICDissipation.hpp"
#include "Schedulers/ReactivateCHCLBonds.hpp"
#include "Schedulers/RemoveMaterialPoint.hpp"
#include "Schedulers/RemoveObstacle.hpp"

#include "Core/MaterialPoint.hpp"

#include "Mth.hpp"

#include <list>

/**
 * @brief Constructor of the MPMbox class
 *
 * Initializes the MPMbox with default values for its fields.
 *
 * The constructor also calls the ExplicitRegistrations() function.
 */
MPMbox::MPMbox() {
  shapeFunction = nullptr;
  oneStep = nullptr;
  planeStrain = false;
  Grid.Nx = 20;
  Grid.Ny = 20;
  tolmass = 1.0e-6;
  gravity.set(0.0, 0.0);
  CHCL.minDEMstep = 5;
  CHCL.rateAverage = 0;
  CHCL.limitTimeStepFactor = 1e-3;
  CHCL.criticalDEMTimeStepFactor = 0.01;
  ratioFLIP = 0.95;
  activePIC = true;

  iconf = 0;
  confPeriod = 5000;

  dt = 0.00001;
  dtInitial = dt;
  t = 0.0;
  result_folder = ".";

  securDistFactor = 2.0;

  splitting = false;
  splitCriterionValue = 2.0;
  MaxSplitNumber = 5;

  id_kn = dataTable.add("kn");
  id_kt = dataTable.add("kt");
  id_en2 = dataTable.add("en2");
  id_mu = dataTable.add("mu");
  id_viscRate = dataTable.add("viscRate");
  id_dn0 = dataTable.add("dn0");
  id_dt0 = dataTable.add("dt0");

  ExplicitRegistrations();
}

/**
 * @brief Destructor of the MPMbox class.
 *
 * This is the default destructor of the MPMbox class, which is
 * responsible for cleaning up the allocated memory.
 *
 * @details
 * The destructor just calls the clean() method, which is
 * responsible for freeing the memory allocated during the
 * initialization of the MPMbox object. This is important to
 * prevent memory leaks.
 */
MPMbox::~MPMbox() { clean(); }

/**
 * @brief Displays the application banner.
 *
 * This function outputs a stylized banner to the console,
 * representing the application's name or logo using ASCII art.
 * It adds a visual separator before and after the banner
 * for better readability.
 */
void MPMbox::showAppBanner() {
  std::cout << std::endl;
  std::cout << "    _/      _/  _/_/_/    _/      _/  _/                         " << std::endl;
  std::cout << "   _/_/  _/_/  _/    _/  _/_/  _/_/  _/_/_/      _/_/    _/    _/" << std::endl;
  std::cout << "  _/  _/  _/  _/_/_/    _/  _/  _/  _/    _/  _/    _/    _/_/   " << std::endl;
  std::cout << " _/      _/  _/        _/      _/  _/    _/  _/    _/  _/    _/  " << std::endl;
  std::cout << "_/      _/  _/        _/      _/  _/_/_/      _/_/    _/    _/   " << std::endl;
  std::cout << std::endl;
}

/**
 * @brief Registers all the necessary classes to the factories.
 *
 * This is a static method that registers all the necessary classes to the factories.
 * It is called by the constructor of the MPMbox class.
 *
 * @details
 * This method is responsible for registering all the necessary classes to the factories,
 * which are used later on in the code to create objects of the registered classes.
 * The classes are registered by calling the RegisterFactoryFunction method of the
 * corresponding factory, and providing a lambda function that returns an instance of
 * the class to be registered.
 */
void MPMbox::ExplicitRegistrations() {

  // BoundaryForceLaw ==========
  Factory<BoundaryForceLaw, std::string>::Instance()->RegisterFactoryFunction(
      "frictionalNormalRestitution", [](void) -> BoundaryForceLaw* { return new frictionalNormalRestitution(); });
  Factory<BoundaryForceLaw, std::string>::Instance()->RegisterFactoryFunction(
      "frictionalViscoElastic", [](void) -> BoundaryForceLaw* { return new frictionalViscoElastic(); });

  // Command ===================
  Factory<Command, std::string>::Instance()->RegisterFactoryFunction(
      "add_MP_ShallowPath", [](void) -> Command* { return new add_MP_ShallowPath(); });
  Factory<Command, std::string>::Instance()->RegisterFactoryFunction("move_MP",
                                                                     [](void) -> Command* { return new move_MP(); });
  Factory<Command, std::string>::Instance()->RegisterFactoryFunction(
      "new_set_grid", [](void) -> Command* { return new new_set_grid(); });
  Factory<Command, std::string>::Instance()->RegisterFactoryFunction(
      "reset_model", [](void) -> Command* { return new reset_model(); });
  Factory<Command, std::string>::Instance()->RegisterFactoryFunction(
      "set_BC_column", [](void) -> Command* { return new set_BC_column(); });
  Factory<Command, std::string>::Instance()->RegisterFactoryFunction(
      "set_BC_line", [](void) -> Command* { return new set_BC_line(); });
  Factory<Command, std::string>::Instance()->RegisterFactoryFunction(
      "set_K0_stress", [](void) -> Command* { return new set_K0_stress(); });
  Factory<Command, std::string>::Instance()->RegisterFactoryFunction(
      "set_MP_grid", [](void) -> Command* { return new set_MP_grid(); });
  Factory<Command, std::string>::Instance()->RegisterFactoryFunction(
      "set_MP_polygon", [](void) -> Command* { return new set_MP_polygon(); });
  Factory<Command, std::string>::Instance()->RegisterFactoryFunction(
      "set_node_grid", [](void) -> Command* { return new set_node_grid(); });
  Factory<Command, std::string>::Instance()->RegisterFactoryFunction(
      "select_tracked_MP", [](void) -> Command* { return new select_tracked_MP(); });
  Factory<Command, std::string>::Instance()->RegisterFactoryFunction(
      "select_controlled_MP", [](void) -> Command* { return new select_controlled_MP(); });
  Factory<Command, std::string>::Instance()->RegisterFactoryFunction(
      "set_uniform_pressure", [](void) -> Command* { return new set_uniform_pressure(); });

  // ConstitutiveModel =========
  Factory<ConstitutiveModel, std::string>::Instance()->RegisterFactoryFunction(
      "CHCL_DEM", [](void) -> ConstitutiveModel* { return new CHCL_DEM(); });
  Factory<ConstitutiveModel, std::string>::Instance()->RegisterFactoryFunction(
      "HookeElasticity", [](void) -> ConstitutiveModel* { return new HookeElasticity(); });
  Factory<ConstitutiveModel, std::string>::Instance()->RegisterFactoryFunction(
      "KelvinVoigt", [](void) -> ConstitutiveModel* { return new KelvinVoigt(); });
  Factory<ConstitutiveModel, std::string>::Instance()->RegisterFactoryFunction(
      "MohrCoulomb", [](void) -> ConstitutiveModel* { return new MohrCoulomb(); });
  Factory<ConstitutiveModel, std::string>::Instance()->RegisterFactoryFunction(
      "VonMisesElastoPlasticity", [](void) -> ConstitutiveModel* { return new VonMisesElastoPlasticity(); });
  Factory<ConstitutiveModel, std::string>::Instance()->RegisterFactoryFunction(
      "SinfoniettaClassica", [](void) -> ConstitutiveModel* { return new SinfoniettaClassica(); });
  Factory<ConstitutiveModel, std::string>::Instance()->RegisterFactoryFunction(
      "SinfoniettaCrush", [](void) -> ConstitutiveModel* { return new SinfoniettaCrush(); });

  // Obstacle ==================
  Factory<Obstacle, std::string>::Instance()->RegisterFactoryFunction("Circle",
                                                                      [](void) -> Obstacle* { return new Circle(); });
  Factory<Obstacle, std::string>::Instance()->RegisterFactoryFunction("Line",
                                                                      [](void) -> Obstacle* { return new Line(); });
  Factory<Obstacle, std::string>::Instance()->RegisterFactoryFunction("Polygon",
                                                                      [](void) -> Obstacle* { return new Polygon(); });

  // OneStep ===================
  Factory<OneStep, std::string>::Instance()->RegisterFactoryFunction(
      "ModifiedLagrangian", [](void) -> OneStep* { return new ModifiedLagrangian(); });
  Factory<OneStep, std::string>::Instance()->RegisterFactoryFunction(
      "UpdateStressFirst", [](void) -> OneStep* { return new UpdateStressFirst(); });
  Factory<OneStep, std::string>::Instance()->RegisterFactoryFunction(
      "UpdateStressLast", [](void) -> OneStep* { return new UpdateStressLast(); });

  // ShapeFunction =============
  Factory<ShapeFunction, std::string>::Instance()->RegisterFactoryFunction(
      "BSpline", [](void) -> ShapeFunction* { return new BSpline(); });
  Factory<ShapeFunction, std::string>::Instance()->RegisterFactoryFunction(
      "Linear", [](void) -> ShapeFunction* { return new Linear(); });
  Factory<ShapeFunction, std::string>::Instance()->RegisterFactoryFunction(
      "RegularQuadLinear", [](void) -> ShapeFunction* { return new RegularQuadLinear(); });

  // Scheduler ==================
  Factory<Scheduler, std::string>::Instance()->RegisterFactoryFunction(
      "GravityRamp", [](void) -> Scheduler* { return new GravityRamp(); });
  Factory<Scheduler, std::string>::Instance()->RegisterFactoryFunction(
      "PICDissipation", [](void) -> Scheduler* { return new PICDissipation(); });
  Factory<Scheduler, std::string>::Instance()->RegisterFactoryFunction(
      "RemoveObstacle", [](void) -> Scheduler* { return new RemoveObstacle(); });
  Factory<Scheduler, std::string>::Instance()->RegisterFactoryFunction(
      "RemoveMaterialPoint", [](void) -> Scheduler* { return new RemoveMaterialPoint(); });
  Factory<Scheduler, std::string>::Instance()->RegisterFactoryFunction(
      "ReactivateCHCLBonds", [](void) -> Scheduler* { return new ReactivateCHCLBonds(); });

  // Spy ========================
  Factory<Spy, std::string>::Instance()->RegisterFactoryFunction("ObstacleTracking",
                                                                 [](void) -> Spy* { return new ObstacleTracking(); });
  Factory<Spy, std::string>::Instance()->RegisterFactoryFunction("Work", [](void) -> Spy* { return new Work(); });
  Factory<Spy, std::string>::Instance()->RegisterFactoryFunction("EnergyBalance",
                                                                 [](void) -> Spy* { return new EnergyBalance(); });
  Factory<Spy, std::string>::Instance()->RegisterFactoryFunction("MeanStress",
                                                                 [](void) -> Spy* { return new MeanStress(); });
  Factory<Spy, std::string>::Instance()->RegisterFactoryFunction("MPTracking",
                                                                 [](void) -> Spy* { return new MPTracking(); });
  Factory<Spy, std::string>::Instance()->RegisterFactoryFunction("ElasticBeamDev",
                                                                 [](void) -> Spy* { return new ElasticBeamDev(); });
}

/**
 * @brief Sets the verbosity level for the logger
 *
 * The verbosity level corresponds to the following levels of logging:
 *  - 0: off
 *  - 1: critical
 *  - 2: error
 *  - 3: warning
 *  - 4: info
 *  - 5: debug
 *  - 6: trace
 *
 * If the given verbosity level is not recognized, the logger will default to info level.
 */
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

/**
 * @brief Cleans up the MPMbox object by releasing allocated resources.
 *
 * This method clears the internal data structures used by the MPMbox,
 * including nodes, elements, material points (MP), obstacles, and models.
 * It deletes dynamically allocated memory for obstacles and constitutive
 * models to prevent memory leaks. After calling this function, the MPMbox
 * object is reset to an empty state.
 */
void MPMbox::clean() {
  nodes.clear();
  Elem.clear();
  MP.clear();

  for (size_t i = 0; i < Obstacles.size(); i++) {
    delete (Obstacles[i]);
  }
  Obstacles.clear();

  std::map<std::string, ConstitutiveModel*>::iterator itModel;
  for (itModel = models.begin(); itModel != models.end(); ++itModel) {
    delete itModel->second;
  }
  models.clear();
}

/**
 * @brief Reads a MPMbox object from a file.
 *
 * This function reads a MPMbox object from a file, which must be a text file
 * containing the information about the nodes, elements, material points, obstacles,
 * models, and other parameters of the MPMbox. The file format is specific to
 * the MPMbox library and is described in the user manual.
 *
 * @param name The name of the file to read from.
 */
void MPMbox::read(const char* name) {
  std::ifstream file(name);
  if (!file) {
    Logger::warn("@MPMbox::read, cannot open file {}", name);
    return;
  }

  std::string token;
  file >> token;
  while (file) {
    if (token[0] == '/' || token[0] == '#' || token[0] == '!')
      getline(file, token);
    else if (token == "result_folder") {
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
    } else if (token == "demavg") {
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
      size_t g1, g2;  // g1 corresponds to MPgroup and g2 to obstacle group
      double value;
      file >> param >> g1 >> g2 >> value;
      dataTable.set(param, g1, g2, value);
    } else if (token == "prescribedVelocity") {  // TODO (V) -> Move it as a command
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
    } else if (token == "model") {
      std::string modelName, modelID;
      file >> modelID >> modelName;
      ConstitutiveModel* CM = Factory<ConstitutiveModel>::Instance()->Create(modelID);
      if (CM != nullptr) {
        models[modelName] = CM;
        CM->key = modelName;
        CM->box = this;
        CM->read(file);
      } else {
        Logger::warn("mode {} is unknown!", modelID);
      }
    } else if (token == "Obstacle") {
      std::string obsName;
      file >> obsName;
      Obstacle* obs = Factory<Obstacle>::Instance()->Create(obsName);
      if (obs != nullptr) {
        obs->read(file);
        Obstacles.push_back(obs);
      } else {
        Logger::warn("Obstacle {} is unknown!", obsName);
      }
    } else if (token == "BoundaryForceLaw") {
      // This has to be defined after defining the obstacles
      if (Obstacles.empty()) {
        Logger::warn("You try to define BoundaryForceLaw BEFORE any Obstacle is set!");
      }
      std::string boundaryName;
      int obstacleGroup;
      file >> boundaryName >> obstacleGroup;
      BoundaryForceLaw* bType = Factory<BoundaryForceLaw>::Instance()->Create(boundaryName);
      for (size_t o = 0; o < Obstacles.size(); o++) {
        if (Obstacles[o]->group == obstacleGroup) {
          Obstacles[o]->boundaryForceLaw = bType;
        }
      }
    } else if (token == "Scheduled") {
      std::string scheduledName;
      file >> scheduledName;
      Scheduler* sch = Factory<Scheduler>::Instance()->Create(scheduledName);
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
      Spy* spy = Factory<Spy>::Instance()->Create(spyName);
      if (spy != nullptr) {
        spy->plug(this);
        spy->read(file);
        Spies.push_back(spy);
      } else {
        Logger::warn("Spy {} is unknown!", spyName);
      }
    } else if (token == "node") {
      size_t nb;
      file >> nb;
      nodes.clear();
      node N;
      for (size_t inode = 0; inode < nb; inode++) {
        file >> N.number >> N.pos;
        nodes.push_back(N);
      }
    } else if (token == "Elem") {
      size_t nb;
      file >> element::nbNodes >> nb;
      Elem.clear();
      element E;
      for (size_t e = 0; e < nb; e++) {

        for (size_t r = 0; r < (size_t)element::nbNodes; r++) {
          file >> E.I[r];
        }
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
        // Il faut changer le sorties suivantes (enlever stressCorrection, ajouter hardeningForce, mettre
        // outOfPlaneStress à cote de stress)
        // Pas maintenant, pour ne pas casser la compatibilité...
        file >> modelName >> P.nb >> P.groupNb >> P.vol0 >> P.vol >> P.density >> P.pos >> P.vel >> P.strain >>
            P.plasticStrain >> P.stress >> P.stressCorrection >> P.splitCount >> P.F >> P.outOfPlaneStress >>
            P.contactf;

        auto itCM = models.find(modelName);
        if (itCM == models.end()) {
          Logger::warn("@MPMbox::read, model {} not found", modelName);
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
    } else {  // it is possible that the keyword corresponds to a command-pluggin
      Command* com = Factory<Command>::Instance()->Create(token);
      if (com != nullptr) {
        com->plug(this);
        com->read(file);
        com->exec();
      } else {
        Logger::warn("@MPMbox::read, what do you mean by '{}'?", token);
      }
    }

    file >> token;
  }  // end while-loop

  // Some checks before running a simulation
  if (!shapeFunction) {
    std::string defaultShapeFunction = "Linear";
    shapeFunction = Factory<ShapeFunction>::Instance()->Create(defaultShapeFunction);
    Logger::info("No ShapeFunction defined, automatically set to 'Linear'");
  }

  if (!oneStep) {
    std::string defaultOneStep = "ModifiedLagrangian";
    oneStep = Factory<OneStep>::Instance()->Create(defaultOneStep);
    Logger::info("No OneStep type defined, automatically set to 'ModifiedLagrangian'");
  }
  dtInitial = dt;

  CHCL.hasDoubleScale = false;
  for (size_t p = 0; p < MP.size(); p++) {
    if (MP[p].isDoubleScale == true) {
      CHCL.hasDoubleScale = true;
      break;
    }
  }
}

/**
 * @brief Read a configuration file.
 *
 * Opens a file named <code>conf<num>.txt</code> in the
 * <code>result_folder</code> directory, and reads in the configuration
 * of the MPMbox object from that file. The configuration file is
 * assumed to have been written by a call to the <code>save</code>
 * method.
 *
 * @param num the number of the configuration file to read.
 */
void MPMbox::read(int num) {
  // Open file
  char name[256];
  snprintf(name, 256, "%s/conf%d.txt", result_folder.c_str(), num);
  Logger::info("Read {}", name);
  read(name);
}

/**
 * @brief Write a configuration file.
 *
 * Writes a configuration file named <code>conf<num>.txt</code> in the
 * <code>result_folder</code> directory, which contains all the
 * information needed to reconstruct the current state of the MPMbox
 * object. The configuration file is written in a format that can be
 * read by a call to the <code>read</code> method.
 *
 * @param name the name of the configuration file to write.
 */
void MPMbox::save(const char* name) {
  std::ofstream file(name);

  file << "# MPM_CONFIGURATION_FILE Version May 2021\n";

  if (planeStrain == true) {
    file << "planeStrain\n";
  }
  file << "oneStepType " << oneStep->getRegistrationName() << '\n';
  file << "result_folder .\n";
  file << "tolmass " << tolmass << '\n';
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
  file << "ShapeFunction " << shapeFunction->getRegistrationName() << '\n';

  for (size_t sc = 0; sc < Scheduled.size(); sc++) {
    file << "Scheduled ";
    Scheduled[sc]->write(file);
  }

  std::map<std::string, ConstitutiveModel*>::iterator itModel;
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
  file << "Nodes " << nodes.size() << '\n';
  for (size_t in = 0; in < nodes.size(); in++) {
    file << nodes[in].q << ' ' << nodes[in].f << ' ' << nodes[in].fb << ' ' << nodes[in].mass << ' ' << nodes[in].xfixed
         << ' ' << nodes[in].yfixed << '\n';
  }

  // Obstacles
  for (size_t iObst = 0; iObst < Obstacles.size(); iObst++) {
    file << "Obstacle " << Obstacles[iObst]->getRegistrationName() << ' ';
    Obstacles[iObst]->write(file);
  }

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
}

/**
 * @brief Saves the current state of the simulation to a file
 *
 * @param num the number of the configuration to be saved
 *
 * The function saves the current state of the simulation to a file
 * with the name <tt>result_folder/conf\*num*.txt</tt>.
 *
 * The state of the simulation consists of the following information:
 * - the nodes of the Eulerian grid
 * - the Material Points (their position, velocity, strain, stress, etc.)
 * - the rigid obstacles
 * - the models used for the Material Points
 *
 * The function also logs the time and the number of Material Points
 * of the simulation.
 */
void MPMbox::save(int num) {
  char name[256];
  snprintf(name, 256, "%s/conf%d.txt", result_folder.c_str(), num);
  Logger::info("Save {}, #MP: {}, Time: {:.6f} ({:.1f}%)", name, MP.size(), t, 100.0 * t / finalTime);
  save(name);
}

/**
 * @brief Initializes the MPMbox by setting up necessary directories and
 *        updating the previous positions of Material Points.
 *
 * This function performs the following actions:
 * - Creates the result folder if it doesn't exist.
 * - Creates individual folders for each tracked Material Point (MP)
 *   for simulations with double scale.
 * - Updates the previous position of each Material Point to the current
 *   position for future reference.
 */
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

  for (size_t p = 0; p < MP.size(); p++) {
    MP[p].prev_pos = MP[p].pos;
  }
}

/**
 * @brief Run the simulation.
 *
 * This function runs the simulation until the final time is reached.
 * It will:
 * - Check if the Material Points are inside the grid area
 * - Perform the simulation steps
 * - Check for convergence requirements
 * - Save the configuration of the simulation at regular intervals
 * - Check for proximity between Material Points
 * - Split Material Points if necessary
 * - Execute/Record the spies at regular intervals
 * - Shutdown the spies at the end of the simulation
 */
void MPMbox::run() {
  START_TIMER("run");

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
            MP[p].PBC->t = t;
            MP[p].PBC->tmax = t;
            MP[p].PBC->saveConf(fname);
          }
        }
      }

      iconf++;
    }

    if (step % proxPeriod == 0 ||
        MP.size() != number_MP_before_any_split) {  // second condition is needed because of the splitting
      checkProximity();
    }

    for (size_t s = 0; s < Scheduled.size(); ++s) {
      Scheduled[s]->check();
    }

    // run a step!
    int ret = oneStep->advanceOneStep(*this);
    if (ret == 1) break;  // returns 1 only in trajectory analyses when contact is lost and normal vel is 1

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
    Spies[s]->end();  // there is often nothing implemented
  }
}

/**
 * Check the proximity of the MPs and obstacles. This function is called every @c proxPeriod steps.
 * It computes the security distance for each MP and obstacle as @c securDistFactor * velocity * dt * @c proxPeriod.
 * It then calls the @c checkProximity function on each obstacle.
 * @see MPMbox::proxPeriod, Obstacle::checkProximity
 */
void MPMbox::checkProximity() {
  START_TIMER("checkProximity");
  // Compute securDist of MPs
  for (size_t p = 0; p < MP.size(); p++) {
    MP[p].securDist = securDistFactor * norm(MP[p].vel) * dt * proxPeriod;
  }

  for (size_t o = 0; o < Obstacles.size(); o++) {
    Obstacles[o]->securDist = securDistFactor * norm(Obstacles[o]->vel) * dt * proxPeriod;
    Obstacles[o]->checkProximity(*this);
  }
}

/**
 * @brief Check if any Material Point is outside the grid before the start of the simulation.
 *
 * This function will check if any Material Point is outside the grid before the start of the simulation. If any
 * Material Point is found to be outside the grid, a warning message will be printed.
 */
void MPMbox::MPinGridCheck() {
  // checking for MP outside the grid before the start of the simulation
  for (size_t p = 0; p < MP.size(); p++) {
    if (MP[p].pos.x > (double)Grid.Nx * Grid.lx || MP[p].pos.x < 0.0 || MP[p].pos.y > (double)Grid.Ny * Grid.ly ||
        MP[p].pos.y < 0.0) {
      Logger::warn("@MPMbox::MPinGridCheck, Check before simulation: MP position (x={}, y={}) is not inside the grid",
                   MP[p].pos.x, MP[p].pos.y);
    }
  }
}

/**
 * @brief Check for convergence conditions.
 *
 * This function checks for several convergence conditions:
 * - Passthrough velocity condition: the timestep should be smaller than the smallest rayon of the MPs divided by the
 * maximum velocity of the MPs.
 * - Collision condition: the timestep should be smaller than the minimum mass of the MPs divided by the maximum normal
 * stiffness of the obstacles.
 * - CFL condition: the timestep should be smaller than the smallest rayon of the MPs divided by the maximum speed of
 * sound of the MPs. If any of these conditions is not satisfied, the timestep is adjusted to half the critical value.
 * @see MPMbox::dt, MPMbox::dtInitial
 */
void MPMbox::convergenceConditions() {
  START_TIMER("convergenceConditions");

  // finding necessary parameters
  double inf = std::numeric_limits<double>::max();
  double YoungMax = -inf;
  double PoissonMax = -inf;
  double rhoMin = inf;
  double rayMin = inf;
  double knMax = -inf;
  double massMin = inf;
  double velMax = -inf;
  std::set<int> groupsMP;
  std::set<int> groupsObs;
  for (size_t p = 0; p < MP.size(); ++p) {
    YoungMax = std::max(MP[p].constitutiveModel->getYoung(), YoungMax);
    PoissonMax = std::max(MP[p].constitutiveModel->getPoisson(), PoissonMax);
    rhoMin = std::min(MP[p].density, rhoMin);
    massMin = std::min(MP[p].mass, massMin);
    velMax = std::max(MP[p].vel * MP[p].vel, velMax);
    rayMin = std::min(MP[p].vol * Mth::invPi, rayMin);
    groupsMP.insert((size_t)(MP[p].groupNb));
  }
  velMax = sqrt(velMax);
  rayMin = sqrt(rayMin);

  if (Obstacles.size() > 0) {
    for (size_t o = 0; o < Obstacles.size(); ++o) {
      groupsObs.insert(Obstacles[o]->group);
    }
  }

  std::set<int>::iterator it;
  std::set<int>::iterator it2;
  for (it = groupsMP.begin(); it != groupsMP.end(); ++it) {
    for (it2 = groupsObs.begin(); it2 != groupsObs.end(); ++it2) {
      if (dataTable.get(id_kn, *it, *it2) > knMax) {
        knMax = dataTable.get(id_kn, *it, *it2);
      }
    }
  }

  // compute the 3 timestep conditions
  double collision_crit_dt = sqrt(massMin / knMax);
  double passthough_crit_dt = collision_crit_dt;
  if (velMax > 1e-6) {
    passthough_crit_dt = rayMin / velMax;
  }
  double cfl_crit_dt;
  if (YoungMax >= 0 && PoissonMax >= 0) {
    double Kmax = YoungMax / (1.0 - 2.0 * PoissonMax);
    cfl_crit_dt = rayMin / sqrt(Kmax / rhoMin);
  } else {
    cfl_crit_dt = passthough_crit_dt;
  }

  // Choosing critical dt as the smallest
  double criticalDt = std::max({passthough_crit_dt, collision_crit_dt, cfl_crit_dt});

  if (step == 0) {
    Logger::debug("Current dt:                        {}", dt);
    Logger::debug("dt_crit/dt (passthrough velocity): {:.3f}", passthough_crit_dt / dt);
    Logger::debug("dt_crit/dt (collision):            {:.3f}", collision_crit_dt / dt);
    Logger::debug("dt_crit/dt (CFL):                  {:.3f}", cfl_crit_dt / dt);
  }

  if (dt > 0.5 * criticalDt) {
    Logger::info("@MPMbox::convergenceConditions, timestep seems too large!");
    dt = 0.5 * criticalDt;
    dtInitial = dt;
    Logger::info("--> Adjusting to {}", dt);
    Logger::debug("dt_crit/dt (passthrough velocity): {:.3f}", passthough_crit_dt / dt);
    Logger::debug("dt_crit/dt (collision):            {:.3f}", collision_crit_dt / dt);
    Logger::debug("dt_crit/dt (CFL):                  {:.3f}", cfl_crit_dt / dt);
  }
}

// =================================================
//  Functions called by the 'OneStep'-type functions
// =================================================

/**
 * Update the velocity gradient for all material points.
 *
 * The velocity gradient of a material point is computed as the sum of the
 * product of the gradient of the shape function and the velocity of the
 * corresponding node. The velocity gradient is stored in the velGrad member
 * variable of each material point.
 *
 * This function is called by the OneStep functions.
 */
void MPMbox::updateVelocityGradient() {
  START_TIMER("updateVelocityGradient");
  size_t* I;
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    for (size_t r = 0; r < element::nbNodes; r++) {
      MP[p].velGrad.xx += (MP[p].gradN[r].x * nodes[I[r]].vel.x);
      MP[p].velGrad.yy += (MP[p].gradN[r].y * nodes[I[r]].vel.y);
      MP[p].velGrad.xy += (MP[p].gradN[r].y * nodes[I[r]].vel.x);
      MP[p].velGrad.yx += (MP[p].gradN[r].x * nodes[I[r]].vel.y);
    }
  }
}

/**
 * @brief Limit the time-step for DEM simulations.
 *
 * If @c CHCL.limitTimeStepFactor is positive, this function limits the time-step
 * @c dt to a value that is a fraction of the critical time-step for DEM simulations.
 * The critical time-step is computed as @c CHCL.limitTimeStepFactor * Rmin / maxi,
 * where @c Rmin is the minimum radius of the MPs and @c maxi is the maximum absolute
 * value of the components of the velocity gradient tensor of the MPs.
 *
 * @see MPMbox::dt, MPMbox::dtInitial, MPMbox::CHCL
 */
void MPMbox::limitTimeStepForDEM() {
  START_TIMER("limitTimeStepForDEM");
  if (CHCL.limitTimeStepFactor <= 0.0) return;

  dt = dtInitial;
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

      if (maxi < 1e-12)
        dtmax = dt;
      else
        dtmax = CHCL.limitTimeStepFactor * MP[p].PBC->Rmin / maxi;

      dt = (dtmax <= dt) ? dtmax : dt;
    }
  }

  Logger::trace("MPM time-step dt = {} at the end limitTimeStepForDEM", dt);
}

/**
 * @brief Updates the transformation gradient F for all material points.
 *
 * The transformation gradient F is computed at each time-step as @c F = (I + dt * L) * F,
 * where @c I is the identity matrix, @c dt is the time-step, @c L is the velocity gradient
 * tensor computed by @c updateVelocityGradient, and @c F is the transformation gradient
 * at the previous time-step.
 *
 * This function is called by the OneStep functions.
 */
void MPMbox::updateTransformationGradient() {
  START_TIMER("updateTransformationGradient");
  updateVelocityGradient();
  if (CHCL.hasDoubleScale == true) limitTimeStepForDEM();

  for (size_t p = 0; p < MP.size(); p++) {
    MP[p].prev_F = MP[p].F;
    MP[p].F = (mat4r::unit() + dt * MP[p].velGrad) * MP[p].F;
  }
}

/**
 * @brief Perform adaptive refinement of material points.
 *
 * This function checks each material point for excessive shearing or deformation
 * and performs splitting if necessary. The transformation gradient F is set to
 * identity when shearing exceeds a predefined limit. If the deformation satisfies
 * certain criteria, the material point is split into two, with properties adjusted
 * accordingly. The split is either along the x or y direction, based on the
 * deformation extent. This function is part of a refinement strategy to enhance
 * simulation accuracy by adapting the discretization dynamically.
 *
 * - If the shearing in F is too large, F is reset to the identity matrix.
 * - Splitting occurs if the deformation extent in one direction exceeds a critical value.
 * - Splits are performed along the axis with the larger deformation extent.
 * - The function also checks for extreme shearing conditions after the splitting criteria.
 */
void MPMbox::adaptativeRefinement() {
  START_TIMER("adaptativeRefinement");
  for (size_t p = 0; p < MP.size(); p++) {
    if (MP[p].splitCount > MaxSplitNumber) continue;

    // setting F to identity if shearing is too large
    if (fabs(MP[p].F.xy) > shearLimit or fabs(MP[p].F.yx) > shearLimit) {
      MP[p].F.xx = 1;
      MP[p].F.xy = 0;
      MP[p].F.yx = 0;
      MP[p].F.yy = 1;
    }

    double XSquaredExtent = (MP[p].F.xx * MP[p].F.xx + MP[p].F.yx * MP[p].F.yx);
    double YSquaredExtent = (MP[p].F.xy * MP[p].F.xy + MP[p].F.yy * MP[p].F.yy);
    double SquaredCrit = splitCriterionValue * splitCriterionValue;

    bool critX = ((XSquaredExtent / YSquaredExtent) >= SquaredCrit);
    bool critY = ((YSquaredExtent / XSquaredExtent) >= SquaredCrit);

    if ((critX || critY) == true) {
      MP[p].splitCount += 1;
      double halfSizeMP = 0.5 * MP[p].size;

      MP[p].mass *= 0.5;
      MP[p].vol *= 0.5;

      // All properties are copied thank to the auto-generated copy-ctor
      MaterialPoint MP2 = MP[p];
      // MP[p] will go to the left or bottom
      // and MP2 will go to the right or top

      if (critX == true) {  // -> left-right splitting
        vec2r sx = MP[p].F * vec2r(halfSizeMP, 0.0);
        MP[p].pos -= 0.5 * sx;
        MP2.pos += 0.5 * sx;

        MP[p].F.xx *= 0.5;
        MP[p].F.yx *= 0.5;

        MP2.F.xx = MP[p].F.xx;
        MP2.F.yx = MP[p].F.yx;

        // MP[p]     MP2
        // 3 - <-2   3-> - 2
        // |     |   |     |
        // 0 - <-1   0-> - 1
        MP[p].corner[1] -= sx;
        MP[p].corner[2] -= sx;
        MP2.corner[0] += sx;
        MP2.corner[3] += sx;

        MP.push_back(MP2);
      } else {  // -> top-bottom splitting
        vec2r sy = MP[p].F * vec2r(0.0, halfSizeMP);
        MP[p].pos -= 0.5 * sy;
        MP2.pos += 0.5 * sy;

        MP[p].F.xy *= 0.5;
        MP[p].F.yy *= 0.5;

        MP2.F.xy = MP[p].F.xy;
        MP2.F.yy = MP[p].F.yy;

        // 3 - 2
        // ^   ^  MP2
        // 0 - 1
        //
        // 3 - 2
        // v   v  MP[p]
        // 0 - 1
        MP[p].corner[2] -= sy;
        MP[p].corner[3] -= sy;
        MP2.corner[0] += sy;
        MP2.corner[1] += sy;

        MP.push_back(MP2);
      }
    }  // end if if ((critX || critY) == true)

    // checking extremeShearing after checking the above criteria
    if (extremeShearing) {
      bool critExtremeShearing =
          (MP[p].F.xx / MP[p].F.xy < extremeShearingval || MP[p].F.yy / MP[p].F.yx < extremeShearingval);
      if (critExtremeShearing) {
        // ... ???
      }
    }

  }  // end for loop over MPs
}

/**
 * @brief Post-process the MPs after time stepping.
 *
 * Compute the smoothed data (vel, stress, velGrad, outOfPlaneStress, pos, strain,
 * rho) from the MPs by using the shape functions. The data is stored in the
 * Data vector.
 *
 * @param Data the vector of ProcessedDataMP where the smoothed data will be stored.
 */
void MPMbox::postProcess(std::vector<ProcessedDataMP>& Data) {
  Data.clear();
  Data.resize(MP.size());

  // Preparation for smoothed data
  size_t* I;
  for (size_t p = 0; p < MP.size(); p++) {
    shapeFunction->computeInterpolationValues(*this, p);
  }

  // Update Vector of node indices
  std::set<size_t> sortedLive;
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    for (size_t r = 0; r < element::nbNodes; r++) {
      sortedLive.insert(I[r]);
    }
  }
  liveNodeNum.clear();
  std::copy(sortedLive.begin(), sortedLive.end(), std::back_inserter(liveNodeNum));

  // Reset nodal mass
  for (size_t n = 0; n < liveNodeNum.size(); n++) {
    nodes[liveNodeNum[n]].mass = 0.0;
    nodes[liveNodeNum[n]].outOfPlaneStress = 0.0;
    nodes[liveNodeNum[n]].vel.reset();
    nodes[liveNodeNum[n]].stress.reset();
  }

  // Nodal mass
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    for (size_t r = 0; r < element::nbNodes; r++) {
      nodes[I[r]].mass += MP[p].N[r] * MP[p].mass;
      nodes[I[r]].outOfPlaneStress += MP[p].N[r] * MP[p].outOfPlaneStress;
    }
  }

  // smooth procedure
  // MP -> nodes
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    for (size_t r = 0; r < element::nbNodes; r++) {
      nodes[I[r]].vel += MP[p].N[r] * MP[p].mass * MP[p].vel / nodes[I[r]].mass;
      nodes[I[r]].stress += MP[p].N[r] * MP[p].mass * MP[p].stress / nodes[I[r]].mass;
    }
  }
  // nodes -> MPs
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    for (size_t r = 0; r < element::nbNodes; r++) {
      Data[p].vel += nodes[I[r]].vel * MP[p].N[r];
      Data[p].stress += nodes[I[r]].stress * MP[p].N[r];
      Data[p].velGrad.xx += (MP[p].gradN[r].x * nodes[I[r]].vel.x);
      Data[p].velGrad.yy += (MP[p].gradN[r].y * nodes[I[r]].vel.y);
      Data[p].velGrad.xy += (MP[p].gradN[r].y * nodes[I[r]].vel.x);
      Data[p].velGrad.yx += (MP[p].gradN[r].x * nodes[I[r]].vel.y);
      Data[p].outOfPlaneStress += MP[p].N[r] * nodes[I[r]].outOfPlaneStress;
    }
    Data[p].pos = MP[p].pos;
    Data[p].strain = MP[p].F;
    Data[p].rho = MP[p].density;
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
