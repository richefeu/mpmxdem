#include "MPMbox.hpp"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

#include "BoundaryForceLaw/BoundaryForceLaw.hpp"
#include "BoundaryForceLaw/frictionalNormalRestitution.hpp"
#include "BoundaryForceLaw/frictionalViscoElastic.hpp"

#include "Commands/Command.hpp"
#include "Commands/add_MP_ShallowPath.hpp"
#include "Commands/move_MP.hpp"
#include "Commands/new_set_grid.hpp"
#include "Commands/reset_model.hpp"
#include "Commands/select_tracked_MP.hpp"
#include "Commands/set_BC_column.hpp"
#include "Commands/set_BC_line.hpp"
#include "Commands/set_K0_stress.hpp"
#include "Commands/set_MP_grid.hpp"
#include "Commands/set_MP_polygon.hpp"
#include "Commands/set_node_grid.hpp"

#include "ConstitutiveModels/CHCL_DEM.hpp"
#include "ConstitutiveModels/ConstitutiveModel.hpp"
#include "ConstitutiveModels/HookeElasticity.hpp"
#include "ConstitutiveModels/MohrCoulomb.hpp"
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

#include "Spies/ObstacleTracking.hpp"
#include "Spies/Spy.hpp"
#include "Spies/Work.hpp"

#include "Core/MaterialPoint.hpp"

#include "Mth.hpp"

#include <list>

MPMbox::MPMbox() {
  shapeFunction = nullptr;
  oneStep = nullptr;
  planeStrain = false;
  Grid.Nx = 20;
  Grid.Ny = 20;
  tolmass = 1.0e-6;
  gravity_max.set(0.0, -9.81);
  gravity.set(0.0, 0.0);
  gravity_incr.set(0.0, 0.0);
  ramp = false;
  CHCL.minDEMstep = 5;
  CHCL.rateAverage = 0;
  // twinConfSave = false;
  ratioFLIP = 1;
  activePIC = false;
  timePIC = 0.0;
  boundary_layer = 0.0;
  ObstaclePlannedRemoval.time = -1.0;
  ObstaclePlannedRemoval.groupNumber = -1;
  switchGravity = false;
  switchGravTime = -1;
  planned_grav.set(0.0, 0.0);

  iconf = 0;
  confPeriod = 5000;

  dt = 0.00001;
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

  console = spdlog::stdout_color_mt("console");
  ExplicitRegistrations();
}

MPMbox::~MPMbox() { clean(); }

void MPMbox::showAppBanner() {
  std::cout << std::endl;
  std::cout << "    _/      _/  _/_/_/    _/      _/  _/                         " << std::endl;
  std::cout << "   _/_/  _/_/  _/    _/  _/_/  _/_/  _/_/_/      _/_/    _/    _/" << std::endl;
  std::cout << "  _/  _/  _/  _/_/_/    _/  _/  _/  _/    _/  _/    _/    _/_/   " << std::endl;
  std::cout << " _/      _/  _/        _/      _/  _/    _/  _/    _/  _/    _/  " << std::endl;
  std::cout << "_/      _/  _/        _/      _/  _/_/_/      _/_/    _/    _/   " << std::endl;
  std::cout << std::endl;
}

void MPMbox::ExplicitRegistrations() {

  // BoundaryForceLaw ===============
  Factory<BoundaryForceLaw, std::string>::Instance()->RegisterFactoryFunction(
      "frictionalNormalRestitution", [](void) -> BoundaryForceLaw* { return new frictionalNormalRestitution(); });
  Factory<BoundaryForceLaw, std::string>::Instance()->RegisterFactoryFunction(
      "frictionalViscoElastic", [](void) -> BoundaryForceLaw* { return new frictionalViscoElastic(); });

  // Command ===============
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

  // ConstitutiveModel ===============
  Factory<ConstitutiveModel, std::string>::Instance()->RegisterFactoryFunction(
      "CHCL_DEM", [](void) -> ConstitutiveModel* { return new CHCL_DEM(); });
  Factory<ConstitutiveModel, std::string>::Instance()->RegisterFactoryFunction(
      "HookeElasticity", [](void) -> ConstitutiveModel* { return new HookeElasticity(); });
  Factory<ConstitutiveModel, std::string>::Instance()->RegisterFactoryFunction(
      "MohrCoulomb", [](void) -> ConstitutiveModel* { return new MohrCoulomb(); });
  Factory<ConstitutiveModel, std::string>::Instance()->RegisterFactoryFunction(
      "VonMisesElastoPlasticity", [](void) -> ConstitutiveModel* { return new VonMisesElastoPlasticity(); });

  // Obstacle ===============
  Factory<Obstacle, std::string>::Instance()->RegisterFactoryFunction("Circle",
                                                                      [](void) -> Obstacle* { return new Circle(); });
  Factory<Obstacle, std::string>::Instance()->RegisterFactoryFunction("Line",
                                                                      [](void) -> Obstacle* { return new Line(); });
  Factory<Obstacle, std::string>::Instance()->RegisterFactoryFunction("Polygon",
                                                                      [](void) -> Obstacle* { return new Polygon(); });

  // OneStep ===============
  Factory<OneStep, std::string>::Instance()->RegisterFactoryFunction(
      "ModifiedLagrangian", [](void) -> OneStep* { return new ModifiedLagrangian(); });
  Factory<OneStep, std::string>::Instance()->RegisterFactoryFunction(
      "UpdateStressFirst", [](void) -> OneStep* { return new UpdateStressFirst(); });
  Factory<OneStep, std::string>::Instance()->RegisterFactoryFunction(
      "UpdateStressLast", [](void) -> OneStep* { return new UpdateStressLast(); });

  // ShapeFunction ===============
  Factory<ShapeFunction, std::string>::Instance()->RegisterFactoryFunction(
      "BSpline", [](void) -> ShapeFunction* { return new BSpline(); });
  Factory<ShapeFunction, std::string>::Instance()->RegisterFactoryFunction(
      "Linear", [](void) -> ShapeFunction* { return new Linear(); });
  Factory<ShapeFunction, std::string>::Instance()->RegisterFactoryFunction(
      "RegularQuadLinear", [](void) -> ShapeFunction* { return new RegularQuadLinear(); });

  // Spy ===============
  Factory<Spy, std::string>::Instance()->RegisterFactoryFunction("ObstacleTracking",
                                                                 [](void) -> Spy* { return new ObstacleTracking(); });
  Factory<Spy, std::string>::Instance()->RegisterFactoryFunction("Work", [](void) -> Spy* { return new Work(); });
}

/*
    trace    =  6
    debug    =  5
    info     =  4
    warn     =  3
    err      =  2
    critical =  1
    off      =  0
*/
void MPMbox::setVerboseLevel(int v) {
  switch (v) {
    case 6:
      console->set_level(spdlog::level::trace);
      break;
    case 5:
      console->set_level(spdlog::level::debug);
      break;
    case 4:
      console->set_level(spdlog::level::info);
      break;
    case 3:
      console->set_level(spdlog::level::warn);
      break;
    case 2:
      console->set_level(spdlog::level::err);
      break;
    case 1:
      console->set_level(spdlog::level::critical);
      break;
    case 0:
      console->set_level(spdlog::level::off);
      break;
    default:
      console->set_level(spdlog::level::info);
      break;
  }
}

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

void MPMbox::read(const char* name) {
  std::ifstream file(name);
  if (!file) {
    console->warn("@MPMbox::read, cannot open file {}", name);
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
    }

    else if (token == "planeStrain") {
      planeStrain = true;
    } else if (token == "tolmass") {
      file >> tolmass;
    } else if (token == "gravity") {
      file >> gravity_max;
    } else if (token == "finalTime") {
      file >> finalTime;
    } else if (token == "confPeriod") {
      file >> confPeriod;
    } else if (token == "proxPeriod") {
      file >> proxPeriod;
    } /*else if (token == "DEMPeriod") {
      file >> DEMPeriod;
    } */
    else if (token == "dt") {
      file >> dt;
    } else if (token == "t") {
      file >> t;
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
    } else if (token == "PICDissipation") {
      file >> ratioFLIP >> timePIC;
      activePIC = true;
    } else if (token == "demavg") {
      file >> CHCL.minDEMstep >> CHCL.rateAverage;
    } /*else if (token == "twinConfSave") {
      twinConfSave = true;
    } */
    else if (token == "ramp") {
      file >> gravity.x >> gravity.y >> gravity_incr.x >> gravity_incr.y;
      ramp = true;
    } else if (token == "gravitySwitch") {
      file >> switchGravTime >> planned_grav.x >> planned_grav.y;
      switchGravity = true;
    } else if (token == "verletCoef") {
      file >> boundary_layer;
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
        CM->read(file);
      } else {
        console->warn("mode {} is unknown!", modelID);
      }
    }

    else if (token == "Obstacle") {
      std::string obsName;
      file >> obsName;
      Obstacle* obs = Factory<Obstacle>::Instance()->Create(obsName);
      if (obs != nullptr) {
        obs->read(file);
        Obstacles.push_back(obs);
      } else {
        console->warn("Obstacle {} is unknown!", obsName);
      }
    } else if (token == "DelObst" || token == "ObstaclePlannedRemoval") {
      file >> ObstaclePlannedRemoval.groupNumber >> ObstaclePlannedRemoval.time;
    } else if (token == "MPPlannedRemoval") {
      MPPlannedRemoval_t MPL;
      file >> MPL.key >> MPL.time;
      MPPlannedRemoval.push_back(MPL);
    } else if (token == "BoundaryForceLaw") {
      // This has to be defined after defining the obstacles
      std::string boundaryName;
      int obstacleGroup;
      file >> boundaryName >> obstacleGroup;
      BoundaryForceLaw* bType = Factory<BoundaryForceLaw>::Instance()->Create(boundaryName);
      for (size_t o = 0; o < Obstacles.size(); o++) {
        if (Obstacles[o]->group == obstacleGroup) {
          Obstacles[o]->boundaryForceLaw = bType;
        }
      }
    }

    else if (token == "Spy") {
      std::string spyName;
      file >> spyName;
      Spy* spy = Factory<Spy>::Instance()->Create(spyName);
      if (spy != nullptr) {
        spy->plug(this);
        spy->read(file);
        Spies.push_back(spy);
      } else {
        console->warn("Spy {} is unknown!", spyName);
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
        file >> modelName >> P.nb >> P.groupNb >> P.vol0 >> P.vol >> P.density >> P.pos >> P.vel >> P.strain >>
            P.plasticStrain >> P.stress >> P.plasticStress >> P.splitCount >> P.F >> P.outOfPlaneStress >> P.contactf;

        auto itCM = models.find(modelName);
        if (itCM == models.end()) {
          console->warn("@MPMbox::read, model {} not found", modelName);
        }
        P.constitutiveModel = itCM->second;
        P.constitutiveModel->init(P);
        P.constitutiveModel->key = modelName;

        P.mass = P.vol * P.density;
        P.size = sqrt(P.vol0);
        MP.push_back(P);
      }
    }

    else {  // it is possible that the keyword corresponds to a command-pluggin
      Command* com = Factory<Command>::Instance()->Create(token);
      if (com != nullptr) {
        com->plug(this);
        com->read(file);
        com->exec();
      } else {
        msg::unknown(token);
      }
    }

    file >> token;
  }  // while-loop

  // Some checks before running a simulation
  if (!shapeFunction) {
    std::string defaultShapeFunction = "Linear";
    shapeFunction = Factory<ShapeFunction>::Instance()->Create(defaultShapeFunction);
    console->info("No ShapeFunction defined, automatically set to 'Linear'");
  }

  if (!oneStep) {
    std::string defaultOneStep = "ModifiedLagrangian";
    oneStep = Factory<OneStep>::Instance()->Create(defaultOneStep);
    console->info("No OneStep type defined, automatically set to 'ModifiedLagrangian'");
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

void MPMbox::read(int num) {
  // Open file
  char name[256];
  snprintf(name, 256, "%s/conf%d.txt", result_folder.c_str(), num);
  console->info("Read {}", name);
  read(name);
}

void MPMbox::save(const char* name) {
  std::ofstream file(name);

  // char name_micro[256];
  // snprintf(name_micro, 256, "%s_micro", name);
  // std::ofstream file_micro(name_micro);

  file << "# MPM_CONFIGURATION_FILE Version May 2021\n";
  // file_micro << "# MP.x MP.y NInt NB TF FF Rmean Vmean VelMean VelMin VelMax VelVar Vsolid Vcell h_xx h_xy h_yx h_yy
  // "
  //               "ReducedPartDistMean" << std::endl;
  if (planeStrain == true) {
    file << "planeStrain\n";
  }
  file << "oneStepType " << oneStep->getRegistrationName() << '\n';
  file << "result_folder .\n";
  file << "tolmass " << tolmass << '\n';
  file << "gravity " << gravity_max << '\n';
  if (ramp) {
    file << "ramp " << gravity.x << " " << gravity.y << " " << gravity_incr.x << " " << gravity_incr.y << '\n';
  }
  if (switchGravity) {
    file << "gravitySwitch " << switchGravTime << " " << planned_grav.x << " " << planned_grav.y << '\n';
  }
  file << "verletCoef " << boundary_layer << '\n';
  file << "demavg " << CHCL.minDEMstep << " " << CHCL.rateAverage << '\n';
  /*if (twinConfSave) {
    file << "twinConfSave" << '\n';
  }*/
  file << "finalTime " << finalTime << '\n';
  file << "proxPeriod " << proxPeriod << '\n';
  file << "confPeriod " << confPeriod << '\n';
  file << "dt " << dt << '\n';
  file << "t " << t << '\n';
  file << "splitting " << splitting << '\n';
  file << "ShapeFunction " << shapeFunction->getRegistrationName() << '\n';
  file << "PICDissipation " << ratioFLIP << " " << timePIC << '\n';

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
  /*
  file << "node " << nodes.size() << '\n';
  for (size_t inode = 0; inode < nodes.size(); inode++) {
    file << nodes[inode].number << ' ' << nodes[inode].pos << '\n';
  }

  file << "Elem " << element::nbNodes << ' ' << Elem.size() << '\n';
  for (size_t iElem = 0; iElem < Elem.size(); iElem++) {
    for (size_t e = 0; e < (size_t)element::nbNodes; e++) {
      file << Elem[iElem].I[e];
      if (e == (size_t)element::nbNodes - 1)
        file << '\n';
      else
        file << ' ';
    }
  }
  */

  // Obstacles
  for (size_t iObst = 0; iObst < Obstacles.size(); iObst++) {
    file << "Obstacle " << Obstacles[iObst]->getRegistrationName() << ' ';
    Obstacles[iObst]->write(file);
  }

  // Material points
  file << "MPs " << MP.size() << '\n';
  for (size_t iMP = 0; iMP < MP.size(); iMP++) {
    file << MP[iMP].constitutiveModel->key << ' ' << MP[iMP].nb << ' ' << MP[iMP].groupNb << ' ' << MP[iMP].vol0 << ' '
         << MP[iMP].vol << ' ' << MP[iMP].density << ' ' << MP[iMP].pos << ' ' << MP[iMP].vel << ' ' << MP[iMP].strain
         << ' ' << MP[iMP].plasticStrain << ' ' << MP[iMP].stress << ' ' << MP[iMP].plasticStress << ' '
         << MP[iMP].splitCount << ' ' << MP[iMP].F << ' ' << MP[iMP].outOfPlaneStress << ' ' << MP[iMP].contactf
         << '\n';

    /*
    if (MP[iMP].isDoubleScale) {
      MP[iMP].PBC->computeSampleData();
      file_micro << MP[iMP].pos.x << " " << MP[iMP].pos.y << " " << MP[iMP].PBC->nbActiveInteractions << " "
                 << MP[iMP].PBC->nbBonds << " " << MP[iMP].PBC->tensfailure << " " << MP[iMP].PBC->fricfailure << " "
                 << MP[iMP].PBC->Rmean << " " << MP[iMP].PBC->Vmean << " " << MP[iMP].PBC->VelMean << " "
                 << MP[iMP].PBC->VelMin << " " << MP[iMP].PBC->VelMax << " " << MP[iMP].PBC->VelVar << " "
                 << MP[iMP].PBC->Vsolid << " " << fabs(MP[iMP].PBC->Cell.h.det()) << " " << MP[iMP].PBC->Cell.h.xx
                 << " " << MP[iMP].PBC->Cell.h.xy << " " << MP[iMP].PBC->Cell.h.yx << " " << MP[iMP].PBC->Cell.h.yy
                 << " " << MP[iMP].PBC->ReducedPartDistMean << std::endl;
    } else {
      file_micro << MP[iMP].pos.x << " " << MP[iMP].pos.y << " " << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " "
                 << 1.0 << " " << 1.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " "
                 << 1.0 << " " << 1.0 << " " << 0.0 << " " << 0.0 << " " << 1.0 << " "
                 << " " << 0.0 << std::endl;
    }
    */
  }
}

void MPMbox::save(int num) {
  // Open file
  char name[256];
  snprintf(name, 256, "%s/conf%d.txt", result_folder.c_str(), num);
  console->info("Save {}, #MP: {}, Time: {:.6f}", name, MP.size(), t);
  save(name);
}

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

void MPMbox::run() {
  START_TIMER("run");
	
  // Check wether the MPs stand inside the grid area
  MPinGridCheck();

  if (!ramp) {
    gravity.set(gravity_max.x, gravity_max.y);
  }
  step = 0;

  while (t <= finalTime) {

    if (ramp) {
      if (fabs(gravity.x) < fabs(gravity_max.x)) {
        gravity.x += gravity_incr.x;
      }
      if (fabs(gravity.y) < fabs(gravity_max.y)) {
        gravity.y += gravity_incr.y;
      }
      if ((fabs(gravity.x) >= fabs(gravity_max.x)) && (fabs(gravity.y) >= fabs(gravity_max.y))) {
        gravity.set(gravity_max.x, gravity_max.y);
        ramp = false;
      }
    }

    if (switchGravity && switchGravTime <= t) {
      gravity_max.set(planned_grav.x, planned_grav.y);
      switchGravity = false;
    }

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

    /*
    if (twinConfSave) {
      if (step % confPeriod == 1) {  // FIXME: c'est bizarre ce truc (à voir ensemble)
        char name[256];
        snprintf(name, 256, "%s/acc%d.txt", result_folder.c_str(), iconf);
        save(name);
      }
    }
    */

    if (step % proxPeriod == 0 ||
        MP.size() != number_MP_before_any_split) {  // second condition is needed because of the splitting
      checkProximity();
    }

    if (activePIC == true) {
      activePIC = timePIC > t;
      if (activePIC == false) {
        console->info("End of PIC damping at time {}", t);
      }
    }

    // run onestep!
    plannedRemovalObstacle();
    plannedRemovalMP();
    int ret = oneStep->advanceOneStep(*this);
    if (ret == 1) break;  // returns 1 only in trajectory analyses when contact is lost and normal vel is 1

    t += dt;
    step++;
  }

  // shutdown the spies
  for (size_t s = 0; s < Spies.size(); ++s) {
    Spies[s]->end();  // there is often nothing implemented
  }
}

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

void MPMbox::MPinGridCheck() {
  // checking for MP outside the grid before the start of the simulation
  for (size_t p = 0; p < MP.size(); p++) {
    if (MP[p].pos.x > (double)Grid.Nx * Grid.lx || MP[p].pos.x < 0.0 || MP[p].pos.y > (double)Grid.Ny * Grid.ly ||
        MP[p].pos.y < 0.0) {
      console->warn("@MPMbox::MPinGridCheck, Check before simulation: MP position (x={}, y={}) is not inside the grid",
                    MP[p].pos.x, MP[p].pos.y);
      // exit(0);
    }
  }
}

// Convergence conditions for MPs interacting with obstacles
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
      if (dataTable.get(id_kn, *it, *it2) > knMax) knMax = dataTable.get(id_kn, *it, *it2);
    }
  }

  // compute the 3 timestep conditions
  double collision_crit_dt = sqrt(massMin / knMax);
  double passthough_crit_dt = collision_crit_dt;
  if (velMax > 1e-6) passthough_crit_dt = rayMin / velMax;
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
    console->debug("Current dt:                        {}", dt);
    console->debug("dt_crit/dt (passthrough velocity): {:.3f}", passthough_crit_dt / dt);
    console->debug("dt_crit/dt (collision):            {:.3f}", collision_crit_dt / dt);
    console->debug("dt_crit/dt (CFL):                  {:.3f}", cfl_crit_dt / dt);
  }

  if (dt > 0.5 * criticalDt) {
    console->info("@MPMbox::convergenceConditions, timestep seems too large!");
    dt = 0.5 * criticalDt;
    dtInitial = dt;
    console->info("--> Adjusting to {}", dt);
    console->debug("dt_crit/dt (passthrough velocity): {:.3f}", passthough_crit_dt / dt);
    console->debug("dt_crit/dt (collision):            {:.3f}", collision_crit_dt / dt);
    console->debug("dt_crit/dt (CFL):                  {:.3f}", cfl_crit_dt / dt);
  }
}

// =================================================
//  Functions called by the 'OneStep'-type functions
// =================================================

// Node velocities (vel = q/m) need to be already updated after calling this function
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

void MPMbox::limitTimeStepForDEM() {
  START_TIMER("limitTimeStepForDEM");
  dt = dtInitial;
  double dtmax = 0.0;

  for (size_t p = 0; p < MP.size(); p++) {
    if (MP[p].isDoubleScale) {
      mat9r VG3D;
      VG3D.xx = MP[p].velGrad.xx;
      VG3D.xy = MP[p].velGrad.xy;
      VG3D.yx = MP[p].velGrad.yx;
      VG3D.yy = MP[p].velGrad.yy;
      VG3D.zz = 1.0;  // assuming plane strain
      VG3D = VG3D * MP[p].PBC->Cell.h;
      double maxi = std::max({fabs(VG3D.xx), fabs(VG3D.xy), fabs(VG3D.xz), fabs(VG3D.yx), fabs(VG3D.yy), fabs(VG3D.yz),
                              fabs(VG3D.zx), fabs(VG3D.zy), fabs(VG3D.zz)});
      if (maxi < 1e-12)
        dtmax = dt;
      else
        dtmax = 1.0e-3 * MP[p].PBC->Rmin / maxi;
      dt = (dtmax <= dt) ? dtmax : dt;
    }
  }

  console->trace("DEM time-step dt = {} at the end limitTimeStepForDEM", dt);
}

void MPMbox::updateTransformationGradient() {
  START_TIMER("updateTransformationGradient");
  updateVelocityGradient();
  if (CHCL.hasDoubleScale == true) limitTimeStepForDEM();

  for (size_t p = 0; p < MP.size(); p++) {
    MP[p].prev_F = MP[p].F;
    MP[p].F = (mat4r::unit() + dt * MP[p].velGrad) * MP[p].F;
  }
}

void MPMbox::plannedRemovalObstacle() {
  START_TIMER("plannedRemovalObstacle");
  if (ObstaclePlannedRemoval.time >= t && ObstaclePlannedRemoval.time <= t + dt) {
    std::vector<Obstacle*> Obs_swap;
    for (size_t i = 0; i < Obstacles.size(); i++) {
      if (Obstacles[i]->group == ObstaclePlannedRemoval.groupNumber) {
        delete (Obstacles[i]);
      } else {
        Obs_swap.push_back(Obstacles[i]);
      }
    }
    Obs_swap.swap(Obstacles);
    Obs_swap.clear();
  }
}

void MPMbox::plannedRemovalMP() {
  START_TIMER("plannedRemovalMP");
  std::vector<MaterialPoint> MP_swap;
  for (size_t j = 0; j < MPPlannedRemoval.size(); j++) {
    if (MPPlannedRemoval[j].time >= t && MPPlannedRemoval[j].time <= t + dt) {
      for (size_t i = 0; i < MP.size(); i++) {
        if (MP[i].constitutiveModel->key != MPPlannedRemoval[j].key) {
          MP_swap.push_back(MP[i]);
        }
      }
      MP_swap.swap(MP);
      MP_swap.clear();
    }
  }
}
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
    }  // if crit

    // checking extremeShearing after checking the above criteria
    if (extremeShearing) {
      bool critExtremeShearing =
          (MP[p].F.xx / MP[p].F.xy < extremeShearingval || MP[p].F.yy / MP[p].F.yx < extremeShearingval);
      if (critExtremeShearing) {
        // ... ???
      }
    }

  }  // for
}

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
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    for (size_t r = 0; r < element::nbNodes; r++) {
      nodes[I[r]].vel += MP[p].N[r] * MP[p].mass * MP[p].vel / nodes[I[r]].mass;
      nodes[I[r]].stress += MP[p].N[r] * MP[p].mass * MP[p].stress / nodes[I[r]].mass;
    }
  }
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
