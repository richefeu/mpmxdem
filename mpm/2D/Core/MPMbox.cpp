#include "MPMbox.hpp"
#include <BoundaryForceLaw/BoundaryForceLaw.hpp>
#include <Commands/Command.hpp>
#include <ConstitutiveModels/ConstitutiveModel.hpp>
#include <Core/MaterialPoint.hpp>
#include <Obstacles/Obstacle.hpp>
#include <OneStep/OneStep.hpp>
#include <ShapeFunctions/ShapeFunction.hpp>
#include <Spies/Spy.hpp>
#include <VtkOutputs/VtkOutput.hpp>

MPMbox::MPMbox() {
  shapeFunction = nullptr;
  oneStep = nullptr;
  oneStepType = "initialOneStep";
  planeStrain = false;
  activeNumericalDissipation = false;

  Grid.Nx = 20;
  Grid.Ny = 20;
  tolmass = 1.0e-6;
  gravity.set(0.0, -9.81);

  nstep = 100000;
  vtkPeriod = 5000;

  dt = 0.00001;
  t = 0.0;
  result_folder = "./RESULT";

  securDistFactor = 2.0;

  splitting = true;
  splittingMore = false;
  splitCriterionValue = 2.0;
  MaxSplitNumber = 5;

  id_kn = dataTable.add("kn");
  id_kt = dataTable.add("kt");
  id_en2 = dataTable.add("en2");
  id_mu = dataTable.add("mu");
  id_viscosity = dataTable.add("viscosity");
}

MPMbox::~MPMbox() {
  /*
  if (shapeFunction) { delete shapeFunction; shapeFunction = nullptr; }

  for (auto &M : models) delete &M;
  models.clear();

  for (auto &O : Obstacles) delete &O;
  Obstacles.clear();
  */
}

void MPMbox::showAppBanner() {
  std::cout << std::endl;
  std::cout << "    _/      _/  _/_/_/    _/      _/  _/                         " << std::endl;
  std::cout << "   _/_/  _/_/  _/    _/  _/_/  _/_/  _/_/_/      _/_/    _/    _/" << std::endl;
  std::cout << "  _/  _/  _/  _/_/_/    _/  _/  _/  _/    _/  _/    _/    _/_/   " << std::endl;
  std::cout << " _/      _/  _/        _/      _/  _/    _/  _/    _/  _/    _/  " << std::endl;
  std::cout << "_/      _/  _/        _/      _/  _/_/_/      _/_/    _/    _/   " << std::endl;
  std::cout << std::endl;
}

void MPMbox::read(const char* name) {
  std::ifstream file(name);
  if (!file) {
    std::cerr << "@MPMbox::read, cannot open file " << name << std::endl;
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
      file >> gravity;
    } else if (token == "nstep") {
      file >> nstep;
      std::cout << "*****'nstep' IS NO LONGER SUPPORTED. USE 'finalTime'*****" << '\n';
      exit(0);
    } else if (token == "finalTime") {
      file >> finalTime;
    } else if (token == "vtkPeriod") {
      file >> vtkPeriod;
    } else if (token == "proxPeriod") {
      file >> proxPeriod;
    } else if (token == "parallelogramMP") {
      file >> parallelogramMP;
    } else if (token == "dt") {
      file >> dt;
    }  // TODO: create a check to see if the block vel for example is too high (i think you have to compare it also to
       // the grid size?)
    else if (token == "t") {
      file >> t;
    } else if (token == "splitting") {
      file >> splitting;
    } else if (token == "splittingExtremeShearing") {
      file >> extremeShearing >> extremeShearingval;
    } else if (token == "splittingMore") {
      file >> splittingMore;
    } else if (token == "splitCriterionValue") {
      file >> splitCriterionValue;
    } else if (token == "shearLimit") {
      file >> shearLimit;
    } else if (token == "MaxSplitNumber") {
      file >> MaxSplitNumber;
    } else if (token == "NumericalDissipation") {
      file >> NumericalDissipation;
      activeNumericalDissipation = true;
    } else if (token == "Surface") {
      int nbPoints;
      file >> nbPoints;
      for (int i = 0; i < nbPoints; i++) {
        vec2r point;
        file >> point;
        surfacePoints.push_back(point);
      }
      save_vtk_surface();
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
        CM->read(file);
      } else {
        std::cerr << "model " << modelID << " is unknown!" << std::endl;
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
        std::cerr << "Obstacle " << obsName << " is unknown!" << std::endl;
      }
    }

    else if (token == "BoundaryForceLaw") {
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
        std::cerr << "Spy " << spyName << " is unknown!" << std::endl;
      }
    } else if (token == "VtkOutput") {
      std::string outName;
      file >> outName;
      VtkOutput* out = Factory<VtkOutput>::Instance()->Create(outName);
      if (out != nullptr) {
        out->plug(this);
        out->read(file);
        VtkOutputs.push_back(out);
      } else {
        std::cerr << "VtkOutput " << outName << " is unknown!" << std::endl;
      }
    } else {
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

  // Display some informations (it can help to find issues)
  std::cout << "Duration of simulation: " << nstep * dt << " sec" << std::endl;

  // Some checks before running a simulation
  if (!shapeFunction) {
    std::string defaultShapeFunction = "Linear";
    shapeFunction = Factory<ShapeFunction>::Instance()->Create(defaultShapeFunction);
    std::cout << "No ShapeFunction defined, automatically set to 'Linear'." << std::endl;
  }

  if (!oneStep) {
    std::string defaultOneStep = "ModifiedLagrangian";
    oneStep = Factory<OneStep>::Instance()->Create(defaultOneStep);
    std::cout << "No OneStep type defined, automatically set to 'ModifiedLagrangian'." << std::endl;
  }

  if (VtkOutputs.empty()) {
    std::cout << "No VtkOutputs defined, the default ones have been automatically added." << std::endl;
    setDefaultVtkOutputs();
  }
}

void MPMbox::setDefaultVtkOutputs() {
  std::vector<std::string> defaultVtkOutputs = {
      "smoothed_velocity", "smoothed_totalStress", /*"meanPressure"*/
  };

  for (size_t s = 0; s < defaultVtkOutputs.size(); s++) {
    VtkOutput* out = Factory<VtkOutput>::Instance()->Create(defaultVtkOutputs[s]);
    if (out != nullptr) {
      out->plug(this);
      VtkOutputs.push_back(out);
    } else {
      std::cerr << "@setDefaultVtkOutputs, vtkOutput " << defaultVtkOutputs[s] << " has not been found" << std::endl;
    }
  }
}

void MPMbox::init(/*const char* name, const char* dconf*/) {
  // If the result folder does not exist, it is created
  fileTool::create_folder(result_folder);
  // PBC3Dbox  box=PBC3Dbox();
  // box.loadConf(dconf);
  for (size_t p = 0; p < MP.size(); p++) {
    MP[p].prev_pos = MP[p].pos;
    // PBC.push_back(PBC3Dbox());
    // PBC[p].loadConf(dconf);
    // V0.reset(0);
    // PBC[p].Load.VelocityControl(V0);
    // PBC[p].enableSwitch = 1;
    // PBC[p].interVerlet = dt / 4.0;
    // PBC[p].interOut = 2 * dt;
    // PBC[p].interConf = 2 * dt;
  }

  // copying input file to results folder
  /*
  std::ifstream src(name, std::ios::binary);
  char newName[256];
  sprintf(newName, "%s/INPUTFILE.sim", result_folder.c_str());
  std::ofstream dst(newName, std::ios::binary);
  dst << src.rdbuf();
  */

  /*
  //good way to set initial vol but bugs the boundary condition when element (cell) is not full
  //adding element number to a list
  std::set<int> elementNumber;

  for (size_t p = 0; p<MP.size();p++) {
          double invL[2];
          invL[0] = 1.0f / Grid.lx;
          invL[1] = 1.0f / Grid.ly;
          MP[p].e = (int)(trunc(MP[p].pos.x * invL[0]) + trunc(MP[p].pos.y * invL[1]) * Grid.Nx);
          elementNumber.insert(MP[p].e);
  }
  std::vector<int> elementNum;
  std::copy(elementNumber.begin(), elementNumber.end(), std::back_inserter(elementNum));  //copying set to a vector to
  facilitate access

  //now comparing each MP to each element while adding matching MP to a temporary list
  std::vector<MaterialPoint*> tempMP;
  double elementVol = Grid.lx*Grid.ly;
  //big double loop. should prob be optimized (happening only once though)
  for (size_t i = 0; i < elementNum.size(); i++){
          tempMP.clear();
          for (size_t p = 0; p<MP.size();p++) { //getting matching MP in the vector and summing their vols
                  if (elementNum[i] == MP[p].e) {
                          tempMP.push_back(&MP[p]);
                  }
          }
          double mpvol = elementVol/tempMP.size();
          for (size_t m = 0; m<tempMP.size();m++){
                  MP[m].vol = mpvol;
                  MP[m].vol0 = mpvol;
          }
  }

  */
  // Output files
  // std::string namelogFile = "global_stream";
  std::string namelogFile2 = "dataStream";

  char name1[256];
  // sprintf(name1, "%s.txt", namelogFile.c_str());
  // logFile.open(name1, std::fstream::app);  // global file
  sprintf(name1, "%s/%s.txt", result_folder.c_str(), namelogFile2.c_str());
  logFile2.open(name1);  // save any kind of data here
}

void MPMbox::run() {
  // Check wether the MPs stand inside the grid area
  MPinGridCheck();

  // *****************************
  int ivtk = 0;
  step = 0;
  // for (step = 0 ; step <= nstep ; step++) {
  while (t < finalTime) {
    // std::cout << "final Time: "<<finalTime << '\n';
    // std::cout << "t: "<<t << '\n';
    // checking cfl (should be improved but works for now)
    try {
      cflCondition();
    } catch (char const* e) {
      std::cerr << "Error in function cflCondition: " << e << std::endl;
    }

    if (step % vtkPeriod == 0) {
      save_vtk("mpm", ivtk);
      save_vtk_obst("obstacle", ivtk);
      ivtk++;
    }
    if (step % proxPeriod == 0 or MP.size() != number_MP) {  // second condition is needed because of the splitting
      checkProximity();
    }

    // numerical dissipation
    // step > 100 is to give the material some time to settle
    if (activeNumericalDissipation == true && step > 500) {
      checkNumericalDissipation();
    }

    // run onestep!
    int ret = oneStep->advanceOneStep(*this);
    if (ret == 1) break;  // returns 1 only in trajectory analyses when contact is lost and normal vel is 1
    for (size_t s = 0; s < Spies.size(); ++s) {
      Spies[s]->end();  // normally there is nothing implemented
    }
    t += dt;
    step++;
  }
}

// This function deactivate the numerical dissipation if all MP-velocities are less than a prescribed value.
void MPMbox::checkNumericalDissipation() {
  // we check the velocity of MPs and if its less than a limit we set activeNumericalDissipation to false
  for (size_t p = 0; p < MP.size(); p++) {
    if (norm(MP[p].vel) > 5e-3) {
      return;
    }
  }
  // if we come here it means vels were small enough
  activeNumericalDissipation = false;
  std::cout << "activeNumericalDissipation is set to 'false'\n";
}

void MPMbox::checkProximity() {
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
    if (MP[p].pos.x > Grid.Nx * Grid.lx or MP[p].pos.x < 0.0 or MP[p].pos.y > Grid.Ny * Grid.ly or MP[p].pos.y < 0.0) {
      std::cerr << "@MPMbox::MPinGridCheck, Check before simulation: MP pos " << MP[p].pos << " is not inside the grid"
                << std::endl;
      exit(0);
    }
  }
}

void MPMbox::cflCondition() {
  // finding necessary parameters
  double inf = 1000000;
  double negInf = -1000000;
  double YoungMax = negInf;
  double rhoMin = inf;
  double knMax = negInf;
  double massMin = inf;
  // FIXME: Mass is recalculated. this changes every timestep (for now we'll only check at the beginning)
  double velMax = negInf;
  std::set<int> groupsMP;
  std::set<int> groupsObs;
  for (size_t p = 0; p < MP.size(); ++p) {
    if (MP[p].constitutiveModel->getYoung() > YoungMax) YoungMax = MP[p].constitutiveModel->getYoung();
    if (MP[p].density < rhoMin) rhoMin = MP[p].density;
    if (MP[p].mass < massMin) massMin = MP[p].mass;
    if (norm(MP[p].vel) > velMax) velMax = norm(MP[p].vel);
    groupsMP.insert((size_t)(MP[p].groupNb));
  }

  if (Obstacles.size() > 0) {
    for (size_t o = 0; o < Obstacles.size(); ++o) {
      groupsObs.insert(Obstacles[o]->group);
    }
  }

  // FIXME: It's not necessarily right since we can have only certain
  // values for g1 and g2
  // getting the knMax
  std::set<int>::iterator it;
  std::set<int>::iterator it2;
  for (it = groupsMP.begin(); it != groupsMP.end(); ++it) {
    for (it2 = groupsObs.begin(); it2 != groupsObs.end(); ++it2) {
      if (dataTable.get(id_kn, *it, *it2) > knMax) knMax = dataTable.get(id_kn, *it, *it2);
    }
  }

  // First condition (MPM) (book vincent)
  double crit_dt1 = 1.0 / sqrt(YoungMax / rhoMin) * (Grid.lx * Grid.ly / (Grid.lx + Grid.ly));

  // Second condition (due to the DEM boundaries)
  double crit_dt2 = Mth::pi * sqrt(massMin / knMax);

  // Third condition (time step uintah user guide)
  double crit_dt3 = Grid.lx / (sqrt(YoungMax / rhoMin) + velMax);

  // Choosing critical dt as the smallest
  double criticalDt;
  if (crit_dt1 > crit_dt2)
    criticalDt = crit_dt2;
  else
    criticalDt = crit_dt1;

  if (criticalDt > crit_dt3) criticalDt = crit_dt3;

  if (step == 0) {
    std::cout << "dt_crit/dt (MPM): " << crit_dt1 / dt << "\ndt_crit/dt  (DEM): " << crit_dt2 / dt
              << "\ndt_crit/dt  (MPM2): " << crit_dt3 / dt << "\nCurrent dt: " << dt << '\n';
  }

  if (dt > criticalDt / 9) {
    std::cout << "\n@MPMbox::cflCondition, timestep seems too large!\n";
    dt = criticalDt / 11.0;
    std::cout << "--> Adjusting to: " << dt << std::endl;
    std::cout << "dt_crit/dt (MPM): " << crit_dt1 / dt << "\ndt_crit/dt  (DEM): " << crit_dt2 / dt
              << "\ndt_crit/dt  (MPM2): " << crit_dt3 / dt << "\nCurrent dt: " << dt << '\n';
  }
}

// =================================================
//  Functions called by the 'OneStep'-type functions
// =================================================

// Node velocities (vel = q/m) need to be already updated after calling this function
void MPMbox::updateTransformationGradient() {
  int* I;
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    MP[p].velGrad.reset();
    for (int r = 0; r < element::nbNodes; r++) {
      MP[p].velGrad.xx += (MP[p].gradN[r].x * nodes[I[r]].vel.x);
      MP[p].velGrad.yy += (MP[p].gradN[r].y * nodes[I[r]].vel.y);
      MP[p].velGrad.xy += (MP[p].gradN[r].y * nodes[I[r]].vel.x);
      MP[p].velGrad.yx += (MP[p].gradN[r].x * nodes[I[r]].vel.y);
    }

    // original code
    MP[p].F = (mat4::unit() + dt * MP[p].velGrad) * MP[p].F;
  }
}

void MPMbox::adaptativeRefinement() {
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
      }
    }

  }  // for
}

void MPMbox::adaptativeRefinementMore()  /// PENDING DEVEL!!!!!!! (DO NOT USE)
{
  for (size_t p = 0; p < MP.size(); p++) {
    if (MP[p].splitCount > MaxSplitNumber) continue;

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
  }    // for
}

// VR: what was the purpose of this function (???)
void MPMbox::weightIncrement() {
  /*
  int numberStepsforIncrement = 100;  //just for testing

  if (step == 0) {
          for (size_t p = 0; p < MP.size(); p++) {
                  MPmassIncrement.push_back(MP[p].mass/numberStepsforIncrement);
                  //MP[p].mass = MPmassIncrement[p];
                  MP[p].mass /= numberStepsforIncrement;
          }
  }

  else if (step < numberStepsforIncrement) {
          for (size_t p = 0; p < MP.size(); p++) {
                  MP[p].mass += MPmassIncrement[p];
          }
          if (step == numberStepsforIncrement -1) std::cout<<"finished progressive weight increment!"<<std::endl;
  }

  */
}

// =======================
//  vtk-storing Routines
// =======================

// Notice that this function is okay only for regular grids
void MPMbox::save_vtk_grid() {
  char fname[256];
  sprintf(fname, "%s/grid.vtk", result_folder.c_str());
  std::cout << "Save " << fname << "... ";
  std::ofstream file(fname);

  // Header
  file << "# vtk DataFile Version 2.0" << std::endl;
  file << "The grid" << std::endl;
  file << "ASCII" << std::endl;

  // Content
  file << "DATASET POLYDATA" << std::endl;
  file << "POINTS " << nodes.size() << " float" << std::endl;
  for (size_t i = 0; i < nodes.size(); ++i) file << nodes[i].pos << " 0" << std::endl;

  file << " " << std::endl;
  file << "POLYGONS " << Elem.size() << " " << (Elem.size()) * 5 << std::endl;
  for (size_t i = 0; i < Elem.size(); ++i) {
    file << "4 " << Elem[i].I[0] << " " << Elem[i].I[1] << " " << Elem[i].I[2] << " " << Elem[i].I[3] << std::endl;
  }
  // TODO: Print current time every time i save a vtk
  std::cout << "done." << std::endl;
}

void MPMbox::save_vtk(const char* base, int num) {
  // Preparation for smoothed data
  int* I;
  for (size_t p = 0; p < MP.size(); p++) {
    shapeFunction->computeInterpolationValues(*this, p);
  }

  // Reset nodal mass
  for (size_t n = 0; n < liveNodeNum.size(); n++) {
    nodes[liveNodeNum[n]].mass = 0.0;
    nodes[liveNodeNum[n]].vel.reset();
    nodes[liveNodeNum[n]].stress.reset();
  }
  // Nodal mass
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    for (int r = 0; r < element::nbNodes; r++) {
      nodes[I[r]].mass += MP[p].N[r] * MP[p].mass;
    }
  }

  // Open file
  char name[256];
  sprintf(name, "%s/%s%d.vtk", result_folder.c_str(), base, num);
  std::cout << "Save " << name << " #MP: " << MP.size() << " Time: " << t << " ... ";
  std::ofstream file(name);

  // Header
  file << "# vtk DataFile Version 2.0" << std::endl;
  file << "State at time " << t << std::endl;
  file << "ASCII" << std::endl;

  if (parallelogramMP == true) {
    file << "DATASET POLYDATA" << std::endl;
    file << "POINTS " << MP.size() * 4 << " float" << std::endl;
    for (size_t p = 0; p < MP.size(); ++p) {
      for (int j = 0; j < 4; ++j) {
        file << MP[p].corner[j] << " 0" << std::endl;
      }
    }

    file << " " << std::endl;
    file << "POLYGONS " << MP.size() << " " << (MP.size()) * 5 << std::endl;
    for (size_t i = 0; i < MP.size(); ++i) {
      file << "4 " << 0 + 4 * i << " " << 1 + 4 * i << " " << 2 + 4 * i << " " << 3 + 4 * i << std::endl;
    }
    /// CELL DATA ==========
    file << std::endl << "CELL_DATA " << MP.size() << std::endl;
  } else {
    file << "DATASET POLYDATA" << std::endl;
    file << "POINTS " << MP.size() << " float" << std::endl;
    for (size_t p = 0; p < MP.size(); ++p) {
      file << MP[p].pos << " 0" << std::endl;
    }
    /// POINT DATA ==========
    file << std::endl << "POINT_DATA " << MP.size() << std::endl;

    // Size of material points (used for display with glyph)
    file << std::endl << "VECTORS radius float" << std::endl;
    for (size_t i = 0; i < MP.size(); ++i) {
      file << sqrt(MP[i].vol) << " 0 0" << std::endl;
    }
  }

  // Save according to the Options (VtkOutputs)
  for (size_t n = 0; n < VtkOutputs.size(); n++) {
    VtkOutputs[n]->save(file);
  }
  std::cout << "done." << std::endl;
}

void MPMbox::save_vtk_obst(const char* base, int numb) {
  std::vector<int> nbNodesOfObstacle;
  std::vector<vec2r> coords;

  for (size_t o = 0; o < Obstacles.size(); o++) {
    int nb = Obstacles[o]->addVtkPoints(coords);
    nbNodesOfObstacle.push_back(nb);
  }

  char name[256];
  sprintf(name, "%s/%s%d.vtk", result_folder.c_str(), base, numb);
  std::ofstream file(name);

  // Header
  file << "# vtk DataFile Version 2.0" << std::endl;
  file << "State at time " << t << std::endl;
  file << "ASCII" << std::endl;

  file << "DATASET UNSTRUCTURED_GRID" << std::endl;
  file << "POINTS " << coords.size() << " float" << std::endl;
  for (size_t i = 0; i < coords.size(); ++i) {
    file << coords[i] << " 0" << std::endl;
  }

  file << " " << std::endl;
  int nbData = 0;
  for (size_t n = 0; n < nbNodesOfObstacle.size(); n++) nbData += nbNodesOfObstacle[n];
  nbData += nbNodesOfObstacle.size();
  file << "CELLS " << Obstacles.size() << " " << nbData << std::endl;
  int num = 0;
  for (size_t i = 0; i < Obstacles.size(); ++i) {
    file << nbNodesOfObstacle[i];
    for (int nn = 0; nn < nbNodesOfObstacle[i]; nn++) {
      file << " " << num++;
    }
    file << std::endl;
  }

  /// CELL TYPES ==========
  file << std::endl << "CELL_TYPES " << Obstacles.size() << std::endl;
  for (size_t i = 0; i < Obstacles.size(); ++i) {
    if (nbNodesOfObstacle[i] == 2)
      file << "3" << std::endl;
    else
      file << "4" << std::endl;
  }
}

void MPMbox::save_vtk_surface() {
  char name[256];
  sprintf(name, "%s/%s.vtk", result_folder.c_str(), "Surface");
  std::ofstream file(name);

  // Header
  file << "# vtk DataFile Version 2.0" << std::endl;
  file << "State at time " << t << std::endl;
  file << "ASCII" << std::endl;

  file << "DATASET UNSTRUCTURED_GRID" << std::endl;
  file << "POINTS " << surfacePoints.size() << " float" << std::endl;
  for (size_t i = 0; i < surfacePoints.size(); ++i) {
    file << surfacePoints[i] << " 0" << std::endl;
  }
  file << " " << std::endl;

  file << "CELLS " << surfacePoints.size() - 1 << " " << (surfacePoints.size() - 1) * 3 << std::endl;
  int counter = 0;
  for (size_t i = 0; i < surfacePoints.size() - 1; ++i) {
    file << 2 << " " << counter << " " << counter + 1 << std::endl;
    counter++;
  }

  /// CELL TYPES ==========
  file << std::endl << "CELL_TYPES " << surfacePoints.size() - 1 << std::endl;
  for (size_t i = 0; i < surfacePoints.size() - 1; ++i) {
    file << "3" << std::endl;
  }
}

// =======================
//  State saving and loading Routines
// =======================

void MPMbox::save_state(const char* base, int num) {
  char name[256];
  sprintf(name, "%s/%s%d.mpm", result_folder.c_str(), base, num);
  std::ofstream file(name);

  for (size_t i = 0; i < MP.size(); i++) {
    file << MP[i].pos << std::endl;
  }
}
