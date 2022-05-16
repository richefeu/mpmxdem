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
#include <list>

MPMbox::MPMbox() {
  shapeFunction = nullptr;
  oneStep = nullptr;
  planeStrain = false;
  activeNumericalDissipation = false;
  DEMPeriod = 5;
  Grid.Nx = 20;
  Grid.Ny = 20;
  tolmass = 1.0e-6;
  gravity_max.set(0.0, -9.81);
  gravity.set(0.0, 0.0);
  gravity_incr.set(0.0, 0.0);
  ramp=false;
  demstable=false;
  stablelength=0.0;
  ViscousDissipation=0;  
  dispacc=false;
  FLIP=1;
  activePIC=false;
  timePIC=0.0;

  iconf = 0;
  confPeriod = 5000;

  dt = 0.00001;
  t = 0.0;
  result_folder = "./RESULT";

  securDistFactor = 2.0;

  splitting = true;
  splitCriterionValue = 2.0;
  MaxSplitNumber = 5;

  id_kn = dataTable.add("kn");
  id_kt = dataTable.add("kt");
  id_en2 = dataTable.add("en2");
  id_mu = dataTable.add("mu");
  id_viscRate = dataTable.add("viscRate");
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
      file >> gravity_max;
    } else if (token == "finalTime") {
      file >> finalTime;
    } else if (token == "confPeriod") {
      file >> confPeriod;
    } else if (token == "proxPeriod") {
      file >> proxPeriod;
    } else if (token == "DEMPeriod") {
      file >> DEMPeriod;
    } else if (token == "dt") {
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
    } else if (token == "NumericalDissipation") {
      file >> NumericalDissipation >> minVd >> EndNd;
      activeNumericalDissipation = true;
    } else if (token == "ViscousDissipation") {
       file >> ViscousDissipation;  
    } else if (token == "PICDissipation") {
       file >> FLIP >>timePIC;
       activePIC=true;  
    }else if (token== "demstable"){
      file >> stablelength;
      demstable=true;
    }else if (token=="acceleration"){
       dispacc=true;
    } else if (token == "ramp") {
       file >> gravity.x >> gravity.y >> gravity_incr.x >> gravity_incr.y ;
       ramp=true;  
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
    else if (token == "DelObst"){
      file >> delnumber >> deltime; 
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
            P.plasticStrain >> P.stress >> P.plasticStress >> P.splitCount >> P.F >> P.sigma3;

        auto itCM = models.find(modelName);
        if (itCM == models.end()) {
          std::cerr << "@MPMbox::read, model " << modelName << " not found" << std::endl;
        }
        P.constitutiveModel = itCM->second;
        P.constitutiveModel->key = modelName;

        P.mass = P.vol * P.density;
        P.size = sqrt(P.vol0);

        MP.push_back(P);
      }
    }

    else {
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
    std::cout << "No ShapeFunction defined, automatically set to 'Linear'." << std::endl;
  }

  if (!oneStep) {
    std::string defaultOneStep = "ModifiedLagrangian";
    oneStep = Factory<OneStep>::Instance()->Create(defaultOneStep);
    std::cout << "No OneStep type defined, automatically set to 'ModifiedLagrangian'." << std::endl;
  }
  dt_init = dt;
}

void MPMbox::save(const char* name) {
  char name_micro[256];
  sprintf(name_micro,"%s_micro",name);
  //std::cout << name_micro << std::endl;
  std::ofstream file(name);
  std::ofstream file_micro(name_micro);

  file << "# MPM_CONFIGURATION_FILE Version May 2021\n";
  file_micro << "# MP.x MP.y NInt NB TF FF Rmean Vmean VelMean VelMin VelMax VelVar Vsolid Vcell h_xx h_xy h_yx h_yy" << std::endl;
  if (planeStrain == true) {
    file << "planeStrain\n";
  }
  file << "oneStepType " << oneStep->getRegistrationName() << '\n';
  file << "result_folder .\n";
  file << "tolmass " << tolmass << '\n';
  file << "gravity " << gravity_max << '\n';
  if (ramp) {
  file << "ramp " << gravity.x << " " << gravity.y << " " <<gravity_incr.x << " " << gravity_incr.y << '\n';
  }
  if (demstable){
  file << "demstable " << stablelength <<'\n';
  }
  if (dispacc){
  file << "acceleration" << '\n';
  }
  file << "finalTime " << finalTime << '\n';
  file << "proxPeriod " << proxPeriod << '\n';
  file << "confPeriod " << confPeriod << '\n';
  file << "dt " << dt << '\n';
  file << "t " << t << '\n';
  file << "splitting " << splitting << '\n';
  file << "ShapeFunction " << shapeFunction->getRegistrationName() << '\n';
  file << "PICDissipation " << FLIP << " " << timePIC << '\n';

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
  // file << "Obstacles " << Obstacles.size() << '\n';
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
         << MP[iMP].splitCount << ' ' << MP[iMP].F << ' ' << MP[iMP].sigma3 <<'\n';
           if (MP[iMP].ismicro){
            MP[iMP].PBC->computeSampleData();
            file_micro << MP[iMP].pos.x                     << " "
                       << MP[iMP].pos.y                     << " "
                       << MP[iMP].PBC->nbActiveInteractions << " "
                       << MP[iMP].PBC->nbBonds              << " "
                       << MP[iMP].PBC->tensfailure          << " "
                       << MP[iMP].PBC->fricfailure          << " "
                       << MP[iMP].PBC->Rmean                << " "
                       << MP[iMP].PBC->Vmean                << " "
                       << MP[iMP].PBC->VelMean              << " "
                       << MP[iMP].PBC->VelMin               << " "
                       << MP[iMP].PBC->VelMax               << " "
                       << MP[iMP].PBC->VelVar               << " "
                       << MP[iMP].PBC->Vsolid               << " "
                       << fabs(MP[iMP].PBC->Cell.h.det())   << " "
                       << MP[iMP].PBC->Cell.h.xx            << " "
                       << MP[iMP].PBC->Cell.h.xy            << " "
                       << MP[iMP].PBC->Cell.h.yx            << " "
                       << MP[iMP].PBC->Cell.h.yy            << std::endl;
       }
  }
}

void MPMbox::save(int num) {
  // Open file
  char name[256];
  sprintf(name, "%s/conf%d.txt", result_folder.c_str(), num);
  std::cout << "Save " << name << " #MP: " << MP.size() << " Time: " << t << '\n';
  save(name);
}

void MPMbox::init() {
  // If the result folder does not exist, it is created
  fileTool::create_folder(result_folder);
  // PBC3Dbox  box=PBC3Dbox();
  // box.loadConf(dconf);
  for (size_t p = 0; p < MP.size(); p++) {
    // MP[p].PBC.saveConf(p,"titi");
    MP[p].prev_pos = MP[p].pos;

    // PBC.push_back(PBC3Dbox());
    // PBC[p].loadConf(dconf);
    // V0.reset(0);
    // PBC[p].Load.VelocityControl(V0);
    // PBC[p].enableSwitch = 1;
    // PBC[p].interVerlet =dt/4.0;
    // PBC[p].interOut =2*dt;
    // PBC[p].interConf =2*dt;
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
  // std::string namelogFile2 = "dataStream";

  // char name1[256];
  // sprintf(name1, "%s.txt", namelogFile.c_str());
  // logFile.open(name1, std::fstream::app);  // global file
  // sprintf(name1, "%s/%s.txt", result_folder.c_str(), namelogFile2.c_str());
  // logFile2.open(name1);  // save any kind of data here
}

void MPMbox::run() {
  // Check wether the MPs stand inside the grid area
  char name[256];
  MPinGridCheck();
  if(!ramp){gravity.set(gravity_max.x,gravity_max.y);}
   //std::cout << "ramp "<< ramp << '\n';
  step = 0;

  while (t < finalTime) {
  if (ramp){
  if (fabs(gravity.x) < fabs(gravity_max.x))
     {gravity.x+=gravity_incr.x;}
  if (fabs(gravity.y) < fabs(gravity_max.y))
     {gravity.y+=gravity_incr.y;}
  if((fabs(gravity.x) >= fabs(gravity_max.x)) && (fabs(gravity.y) >=fabs(gravity_max.y)))
    { gravity.set(gravity_max.x,gravity_max.y); 
      ramp=false;}
   }
   //std::cout << "g.x "<< gravity.x <<" g.y " << gravity.y << '\n';
   //std::cout << "ramp "<< ramp << '\n';
    // std::cout << "final Time: "<<finalTime << '\n';
    // std::cout << "t: "<<t << '\n';

    // checking cfl (should be improved but works for now)
    //try {
      cflCondition();
    //} catch (char const* e) {
    //  std::cout << "Error testing cfl: " << e << std::endl;
    //}

    if (step % confPeriod == 0) {
      save(iconf);
      iconf++;
    } 
    if (dispacc){
      if (step % confPeriod == 1) {
         sprintf(name, "%s/acc%d.txt", result_folder.c_str(), iconf);
         save(name);
      }
    }
    if (step % proxPeriod == 0 || MP.size() != number_MP) {  // second condition is needed because of the splitting
      checkProximity();
    }

    // numerical dissipation
    // step > 100 is to give the material some time to settle
    if (activeNumericalDissipation == true && step > 100) {
      checkNumericalDissipation(minVd,EndNd);
    }
    if (activePIC==true){
      activePIC = timePIC>t;
      if (activePIC==false){
        printf("end of PIC damping at time %f", t);
      }
    }

    // run onestep!
    int ret = oneStep->advanceOneStep(*this);
    if (ret == 1) break;  // returns 1 only in trajectory analyses when contact is lost and normal vel is 1

    t += dt;
    step++;
  }

  for (size_t s = 0; s < Spies.size(); ++s) {
    Spies[s]->end();  // there is often nothing implemented
  }

  std::cout << "MPMbox::run done" << '\n';
}

// This function deactivates the numerical dissipation if all MP-velocities are less than a prescribed value.
void MPMbox::checkNumericalDissipation(double vmin,double EndNd) {
  // we check the velocity of MPs and if its less than a limit we set activeNumericalDissipation to false
  for (size_t p = 0; p < MP.size(); p++) {
    if (norm(MP[p].vel) > vmin && t< EndNd) {
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
 
  // Second condition (due to the DEM boundaries)
  double crit_dt2 = Mth::pi * sqrt(massMin / knMax);
  double crit_dt1;
  double crit_dt3;
  if (YoungMax>=0){
    // First condition (MPM) (book vincent)
    crit_dt1 = 1.0 / sqrt(YoungMax / rhoMin) * (Grid.lx * Grid.ly / (Grid.lx + Grid.ly));

    // Third condition (time step uintah user guide)
    crit_dt3 = Grid.lx / (sqrt(YoungMax / rhoMin) + velMax);
  }
  else{

    crit_dt1 = 0.5*Grid.lx / (Grid.lx+velMax);
    crit_dt3=crit_dt1;
  }

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
    dt_init=dt;
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
    for (int r = 0; r < element::nbNodes; r++) {
      MP[p].velGrad.xx += (MP[p].gradN[r].x * nodes[I[r]].vel.x);
      MP[p].velGrad.yy += (MP[p].gradN[r].y * nodes[I[r]].vel.y);
      MP[p].velGrad.xy += (MP[p].gradN[r].y * nodes[I[r]].vel.x);
      MP[p].velGrad.yx += (MP[p].gradN[r].x * nodes[I[r]].vel.y);
    }
  }
}

void MPMbox::DEMfinalTime() {
  dt = dt_init;
  float dtmax = 0.0;
  int micnb=0;
  for (size_t p = 0; p < MP.size(); p++) {
    if (MP[p].ismicro) {
       VG3D.reset();
       VG3D.xx = MP[p].velGrad.xx;
       VG3D.xy = MP[p].velGrad.xy;
       VG3D.yx = MP[p].velGrad.yx;
       VG3D.yy = MP[p].velGrad.yy;
       VG3D.zz = 1.0; // assuming plane strain
       VG3D=VG3D*MP[p].PBC->Cell.h;
      dtmax = 1e-3 * MP[p].PBC->Rmin /(VG3D.maxi());
      dt = (dtmax <= dt) ? dtmax : dt;
      micnb+=1;
    }
  }
  if(micnb>0){
     std::cout << "final DEM time-step " << dt << std::endl;
  }
  // original code
  for (size_t p = 0; p < MP.size(); p++) {
    MP[p].prev_F = MP[p].F;
    MP[p].F = (mat4::unit() + dt * MP[p].velGrad) * MP[p].F;
    //MP[p].F = MP[p].F + dt * MP[p].velGrad; 
  }
}
void MPMbox::DeleteObject(){
  if (deltime>=t && deltime <= t+dt){
      for (size_t i = 0; i < Obstacles.size(); i++) {
        if (Obstacles[i]->group==delnumber){
          delete (Obstacles[i]); 
          Obstacles.erase(Obstacles.begin()+i);
        }
     }
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
        // ... ???
      }
    }

  }  // for
}

void MPMbox::postProcess(std::vector<ProcessedDataMP>& Data) {
  Data.clear();
  Data.resize(MP.size());

  // Preparation for smoothed data
  int* I;
  for (size_t p = 0; p < MP.size(); p++) {
    shapeFunction->computeInterpolationValues(*this, p);
  }

  // Update Vector of node indices
  std::set<int> sortedLive;
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    for (int r = 0; r < element::nbNodes; r++) {
      sortedLive.insert(I[r]);
    }
  }
  liveNodeNum.clear();
  std::copy(sortedLive.begin(), sortedLive.end(), std::back_inserter(liveNodeNum));

  // Reset nodal mass
  for (size_t n = 0; n < liveNodeNum.size(); n++) {
    nodes[liveNodeNum[n]].mass  = 0.0;
    nodes[liveNodeNum[n]].sigma3= 0.0;
    nodes[liveNodeNum[n]].vel.reset();
    nodes[liveNodeNum[n]].stress.reset();
  }

  // Nodal mass
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    for (int r = 0; r < element::nbNodes; r++) {
      nodes[I[r]].mass += MP[p].N[r] * MP[p].mass;
      nodes[I[r]].sigma3 += MP[p].N[r] * MP[p].sigma3;
    }
  }

  // smoothed velocities
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    for (int r = 0; r < element::nbNodes; r++) {
      nodes[I[r]].vel += MP[p].N[r] * MP[p].mass * MP[p].vel / nodes[I[r]].mass;
      nodes[I[r]].stress += MP[p].N[r] * MP[p].mass * MP[p].stress / nodes[I[r]].mass;
    }
  }
  for (size_t p = 0; p < MP.size(); p++) {
    I = &(Elem[MP[p].e].I[0]);
    for (int r = 0; r < element::nbNodes; r++) {
      Data[p].vel += nodes[I[r]].vel * MP[p].N[r];
      Data[p].stress += nodes[I[r]].stress * MP[p].N[r];
      Data[p].velGrad.xx += (MP[p].gradN[r].x * nodes[I[r]].vel.x);
      Data[p].velGrad.yy += (MP[p].gradN[r].y * nodes[I[r]].vel.y);
      Data[p].velGrad.xy += (MP[p].gradN[r].y * nodes[I[r]].vel.x);
      Data[p].velGrad.yx += (MP[p].gradN[r].x * nodes[I[r]].vel.y);
      Data[p].sigma3     +=  MP[p].N[r] * nodes[I[r]].sigma3;
    }
    Data[p].pos=MP[p].pos;
    Data[p].strain=MP[p].F;
    Data[p].rho=MP[p].density;
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


/*
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
*/
