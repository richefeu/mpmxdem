#include "MPMbox.hpp"
#include "../Commands/Command.hpp"
#include "../ShapeFunctions/ShapeFunction.hpp"
#include "../ConstitutiveModels/ConstitutiveModel.hpp"
#include "../OneStep/OneStep.hpp"

MPMbox::MPMbox()
{

	shapeFunction = nullptr;
	oneStep = nullptr;
	oneStepType = "initialOneStep";
	planeStrain = false;

	//lgrid = 1.;
	Grid.Nx = 20;
	Grid.Ny = 20;
	Grid.Nz = 20;
	tolmass = 1.e-6;
	gravity.set(0.0, -9.81, 0);

	nstep     = 100000;
	vtkPeriod = 5000;

	dt  = 0.00001;
	t   = 0.;
	result_folder = "./RESULT";

	securDistFactor = 2.0;

	logFile.open("dataStream.txt");
	//logFile1.open("dataStream1.txt");
	id_kn = dataTable.add("kn");
	id_kt = dataTable.add("kt");
	id_en2 = dataTable.add("en2");
	id_mu = dataTable.add("mu");
}

MPMbox::~MPMbox()
{
	if (shapeFunction) { delete shapeFunction; shapeFunction = 0; }

	// temporaire (le temps d'avoir une liste de models)
	if (!MP.empty() && MP[0].constitutiveModel) delete MP[0].constitutiveModel;

}

void MPMbox::showAppBanner ()
{
	std::cout << std::endl;
	std::cout << "    _/      _/  _/_/_/    _/      _/  _/                         " << std::endl;
	std::cout << "   _/_/  _/_/  _/    _/  _/_/  _/_/  _/_/_/      _/_/    _/    _/" << std::endl;
	std::cout << "  _/  _/  _/  _/_/_/    _/  _/  _/  _/    _/  _/    _/    _/_/   " << std::endl;
	std::cout << " _/      _/  _/        _/      _/  _/    _/  _/    _/  _/    _/  " << std::endl;
	std::cout << "_/      _/  _/        _/      _/  _/_/_/      _/_/    _/    _/   " << std::endl;
	std::cout << std::endl;
}

void MPMbox::read (const char * name)
{
	std::ifstream file(name);
	if (!file) {
		std::cerr << "@read, cannot open file " << name << std::endl;
		return;
	}
	std::string token;
	file >> token;
	while (file) {
		if (token[0] == '/' || token[0] == '#' || token[0] == '!') getline(file, token);
		else if (token == "result_folder") {
			file >> result_folder;
			fileTool::create_folder(result_folder);
		}
		else if (token == "oneStepType") {
			std::string typeOneStep;
			file >> typeOneStep;
			if (oneStep) { delete oneStep; oneStep = nullptr; }
			oneStep = Factory<OneStep>::Instance()->Create(typeOneStep);
		}
		else if (token == "planeStrain") { planeStrain = true; }
		else if (token == "tolmass") { file >> tolmass; }
		else if (token == "gravity") { file >> gravity; }
		else if (token == "nstep") {
			file >> nstep;
			std::cout << "*****'nstep' IS NO LONGER SUPPORTED. USE 'finalTime'*****" << '\n';
			exit(0);
			}
		else if (token == "finalTime") { file >> finalTime; }
		else if (token == "vtkPeriod") { file >> vtkPeriod; }
		else if (token == "proxPeriod") { file >> proxPeriod; }
		else if (token == "dt") { file >> dt; }
		else if (token == "t") { file >> t; }
		else if (token == "set") {
			std::string param;
			size_t g1, g2;  //g1 corresponds to MPgroup and g2 to obstacle group
			double value;
			file >> param >> g1 >> g2 >> value;
			dataTable.set(param, g1, g2, value);
		}

		else if (token == "ShapeFunction") {
			std::string shapeFunctionName;
			file >> shapeFunctionName;
			if (shapeFunction) { delete shapeFunction; shapeFunction = nullptr; }
			shapeFunction = Factory<ShapeFunction>::Instance()->Create(shapeFunctionName);
		}

		else if (token == "model") {
			std::string modelName, modelID;
			file >> modelID >> modelName;
			ConstitutiveModel *CM = Factory<ConstitutiveModel>::Instance()->Create(modelID);
			if (CM != nullptr) {
				models[modelName] = CM;
				CM->read(file);
			}
			else { std::cerr << "model " << modelID << " is unknown!" << std::endl; }
		}

		else if (token == "blender_Obstacle") {
			std::string obsName;
			file >> obsName;
			blender_Obstacle * obs = Factory<blender_Obstacle>::Instance()->Create(obsName);
			//i have to loop over the file out here and if it finds plane2 it adds
			//an object
			if (obs != nullptr) {
				obs->read(file, result_folder);
				// obs->deleteIfInside(*this);
				blender_Obstacles.push_back(obs);
				obstacleGroups.insert(obs->group);
			}
			else { std::cerr << "Obstacle " << obsName << " is unknown!" << std::endl; }
		}

		else if (token == "Obstacle") {
			std::string obsName;
			file >> obsName;
			Obstacle * obs = Factory<Obstacle>::Instance()->Create(obsName);

			if (obs != nullptr) {
				obs->read(file);
				obs->deleteIfInside(*this);
				Obstacles.push_back(obs);
				obstacleGroups.insert(obs->group);
			}
			else { std::cerr << "Obstacle " << obsName << " is unknown!" << std::endl; }
		}

		else if (token == "set_grid") {
			std::cout << "*****USE 'new_set_grid' INSTEAD OF 'set_grid'*****" << '\n';
			int nbElemX, nbElemY, nbElemZ;
			double lx, ly, lz;
			file >> nbElemX >> nbElemY >> nbElemZ >> lx >> ly >> lz;
			set_grid(nbElemX, nbElemY, nbElemZ, lx, ly, lz);
		}

		else if (token == "set_BC_line") {
			int line_num, column0, column1, depth0, depth1;
			bool Xfixed, Yfixed;
			file >> line_num >> column0 >> column1 >> depth0 >> depth1 >> Xfixed >> Yfixed;
			set_BC_line(line_num, column0, column1, depth0, depth1, Xfixed, Yfixed);
		}
		else if (token == "set_BC_column") {
			int column_num, line0, line1, depth0, depth1;
			bool Xfixed, Yfixed;
			file >> column_num >> line0 >> line1 >> depth0 >> depth1 >> Xfixed >> Yfixed;
			set_BC_column(column_num, line0, line1, depth0, depth1, Xfixed, Yfixed);
		}

		else if (token == "move_MP") {
			double x0; double y0; double z0; double dx; double dy; double dz; double thetaZ; double thetaY;
			file >> x0 >> y0 >> z0 >> dx >> dy >> dz >> thetaZ >> thetaY;
			move_MP(x0, y0, z0, dx, dy, dz, thetaZ, thetaY);
		}

		else if (token == "set_properties") {
			std::cout << "*****PROPERTIES ARE NO LONGER SET WITH 'set_properties'*****" << '\n';
			// double mu1, K1, e21, Kt1;
			// file >> mu1 >> K1 >> e21>>Kt1;
			// set_properties(mu1, K1, e21, Kt1);
		}

		else if (token == "set_K0_stress") {
			double nu, rho0;
			file >> nu >> rho0;
			set_K0_stress(nu, rho0);
		}

		else if (token == "actualMP") {
			file >> actualMPflag;
		}

		else {
			Command * com = Factory<Command>::Instance()->Create(token);
			if (com != nullptr) {
				com->plug(this);
				com->read(file);
				com->exec();
			}
			else {
				//msg::unknown(token);
				//std::cerr << msg::bg_red() << "Command " << token << " is unknown!" << msg::normal() << std::endl;
				std::cout<<"Command "<<token<<" unknown!"<<std::endl;
			}
		}

		file >> token;
	}

	// Some checks before computation
	if (!shapeFunction) {
		std::string defaultShapeFunction= "RegularQuadLinear";
		shapeFunction = Factory<ShapeFunction>::Instance()->Create(defaultShapeFunction);
		std::cout << "No ShapeFunction defined, automatically set to 'RegularQuadLinear'." << std::endl;
	}
	if (!oneStep) {
		std::string defaultOneStep= "ModifiedLagrangian";
		oneStep = Factory<OneStep>::Instance()->Create(defaultOneStep);
		std::cout << "No OneStep type defined, automatically set to 'ModifiedLagrangian'." << std::endl;
	}
}

void MPMbox::init(const char * name) {
	// fileTool::create_folder(result_folder);
	for (size_t p = 0; p< MP.size(); p++){
		MP[p].prev_pos = MP[p].pos;
	}

	//copying input file to results folder
    std::ifstream  src(name, std::ios::binary);
	char newName[256];
	sprintf (newName, "%s/INPUTFILE.sim", result_folder.c_str());
    std::ofstream  dst(newName,  std::ios::binary);
    dst << src.rdbuf();

}

void MPMbox::run ()
{
	// *****Presimulation checks*****
	MPinGridCheck();

	//*******************************
	int ivtk = 0;
	// for (step = 0 ; step <= nstep ; step++) {
	step = 0;
	while(t < finalTime) {
		// checking cfl
		try {
			cflCondition();
		}
		catch (char const * e) {
			std::cout<<"Error testing cfl: "<<e<<std::endl;
		}

		if (step%vtkPeriod == 0) {
			save_vtk("mpm",ivtk);
			save_MPpos("mpPos", ivtk);
			std::set<int>::iterator it;
			for (it = obstacleGroups.begin(); it!=obstacleGroups.end();it++){
				int obstacleNb = *it;

				save_vtk_obst("obstacle", ivtk, obstacleNb);
			}
			ivtk++;
		}

		if (step % proxPeriod == 0) {
			checkProximity();
		}
		oneStep->advanceOneStep(*this);
		t += dt;  //time update is now here
		step++;

		// if (oneStepType == "modifiedLagrangian") OneStepModifiedLagrangian();
		// else OneStep();
	}
}

void MPMbox::MPinGridCheck() {
	// checking for MP outside the grid before the start of the simulation
	for (size_t p = 0; p < MP.size(); p++) {
		if(MP[p].pos.x > Grid.Nx*Grid.lx or  MP[p].pos.x < 0.0
		or MP[p].pos.y > Grid.Ny*Grid.ly or  MP[p].pos.y < 0.0
		or MP[p].pos.z > Grid.Nz*Grid.lz or  MP[p].pos.z < 0.0)
		{
			std::cerr << "@MPMbox::MPinGridCheck, Check before simulation: MP pos "<<MP[p].pos<<" is not inside the grid"<< std::endl;
			exit(0);
		}
	}
}

void MPMbox::cflCondition() {
	// finding necessary parameters
	double inf = 1000000;
	double negInf = -1000000;
	double YoungMax = negInf;
	// double YoungMin = inf;
	double rhoMin = inf;
	// double rhoMax = negInf;
	// double knMin = inf;
	double knMax = negInf;
	// double massMax = negInf;  //FIXME: Mass is recalculated. this changes every timestep (for now we'll only check at the beginning)
	double massMin = inf;  //FIXME: Mass is recalculated. this changes every timestep (for now we'll only check at the beginning)
	double velMax = negInf;
	std::set<int> groupsMP;
	std::set<int> groupsObs;
	for (size_t p = 0; p < MP.size(); ++p) {
		if (MP[p].constitutiveModel->getYoung() > YoungMax) YoungMax = MP[p].constitutiveModel->getYoung();
		// if (MP[p].constitutiveModel->getYoung() < YoungMin) YoungMin = MP[p].constitutiveModel->getYoung();
		if (MP[p].density < rhoMin) rhoMin = MP[p].density;
		// if (MP[p].density > rhoMax) rhoMax = MP[p].density;
		// if (MP[p].mass > massMax) massMax = MP[p].mass;
		if (MP[p].mass < massMin) massMin = MP[p].mass;
		if (norm(MP[p].vel) > velMax) velMax = norm(MP[p].vel);
		groupsMP.insert((size_t)(MP[p].groupNb));
	}

	if (Obstacles.size() > 0){
		for (size_t o = 0 ; o < Obstacles.size() ; ++o) {
			groupsObs.insert(Obstacles[o]->group);
		}
	}

	// FIXME: It's not necessarily right since we can have only certain
	// values for g1 and g2
	// getting the knMax
	std::set<int>::iterator it;
	std::set<int>::iterator it2;
	for (it = groupsMP.begin(); it!= groupsMP.end(); ++it) {
		for (it2 = groupsObs.begin(); it2!= groupsObs.end(); ++it2) {
			// if (dataTable.get(id_kn, *it, *it2) < knMin) knMin = dataTable.get(id_kn, *it, *it2);
			if (dataTable.get(id_kn, *it, *it2) > knMax) knMax = dataTable.get(id_kn, *it, *it2);
		}
	}



	// First condition (MPM) (book vincent)
	// double crit_dt1 = sqrt(rhoMin/YoungMax) * (Grid.lx * Grid.ly/(Grid.lx + Grid.ly));
	double crit_dt1 = 1/sqrt(YoungMax/rhoMin) * (Grid.lx * Grid.ly/(Grid.lx + Grid.ly));
	// Second condition (due to the DEM boundaries) https://yade-dem.org/doc/formulation.html
	// double crit_dt2 = Mth::pi * sqrt(knMin/massMax);
	double crit_dt2 = Mth::pi * sqrt(massMin/knMax);
	//Third condition (time step uintah user guide)
	double crit_dt3 = Grid.lx/(sqrt(YoungMax/rhoMin) + velMax);
	// Choosing critical dt
	double criticalDt;
	if (crit_dt1 > crit_dt2) criticalDt = crit_dt2;
	else criticalDt = crit_dt1;

	if (criticalDt > crit_dt3) criticalDt = crit_dt3;

	// std::cout << "velMax: "<<velMax<<" dx: "<<Grid.lx<<" wave: "<<sqrt(YoungMin/rhoMax);
	// std::cout << " crit_dt3: "<<crit_dt3 << '\n';

	if (step == 0) std::cout<<"Min dt (MPM): "<<crit_dt1/11<< "  Min dt (DEM): "<<crit_dt2/11<< "  Min dt (MPM2): "<<crit_dt3/11<< "  Current dt: "<<dt<<std::endl;
	if (dt > criticalDt/9) {
		std::cout<<"\n*****@MPMbox::cflCondition, The timestep is too large!***** \nAdjusting to: "<<criticalDt/11<<std::endl;
		std::cout<<"Min dt (MPM): "<<crit_dt1/11<< "  Min dt (DEM): "<<crit_dt2/11<< "  Min dt (MPM2): "<<crit_dt3/11<< "  Current dt: "<<dt<<std::endl;
		dt = criticalDt/11;
		// exit(0);
	}
}

void MPMbox::checkProximity()
{
	for (size_t p = 0 ; p < MP.size() ; p++) {
		MP[p].securDist = securDistFactor * norm(MP[p].vel) * dt * proxPeriod;
	}

	for (size_t o = 0 ; o < Obstacles.size() ; o++) {
		Obstacles[o]->checkProximity(*this);
	}

	for (size_t o = 0 ; o < blender_Obstacles.size() ; o++) {
		blender_Obstacles[o]->checkProximity(*this);
	}
}


// ===============================
//  Functions called in OneStep!!!
// ===============================


void MPMbox::blenderBoundaryConditions(){

	//new implementation

	double kn, kt, en2, mu;
	size_t g1, g2;
	for (size_t o = 0 ; o < blender_Obstacles.size() ; ++o) {
		for (size_t nn = 0 ; nn < blender_Obstacles[o]->Neighbors.size() ; ++nn) {
			// if (o==1) std::cout<<blender_Obstacles[o]->Neighbors.size()<<std::endl;
			// std::cout<<"o =  "<<o <<" neigh: "<<blender_Obstacles[o]->Neighbors.size()<<std::endl;
			size_t pn = blender_Obstacles[o]->Neighbors[nn].PointNumber;
			double dn = 0.0;
      		double sumfn = 0.0;

      		if ( blender_Obstacles[o]->touch(MP[pn],0,dn) ) {

				// std::cout<<"Contact! "<<o<<std::endl;
				g1 = (size_t)(MP[pn].groupNb);
				g2 = blender_Obstacles[o]->group;
				kn = dataTable.get(id_kn, g1, g2);
				kt = dataTable.get(id_kt, g1, g2);
				en2 = dataTable.get(id_en2, g1, g2);
				mu = dataTable.get(id_mu, g1, g2);

				vec3r N, T1, T2;
				blender_Obstacles[o]->getContactFrame(MP[pn], N, T1, T2);
				//TODO: Update neighbor class like in 2D and update the BC correspondingly (also the obstacles)
				// === Normal force
				if (dn < 0.0) { // overlapping
					double delta_dn = dn - blender_Obstacles[o]->Neighbors[nn].dn[0];
					if (delta_dn > 0.0) {      // Unloading
						// if (Obstacles[o]->group == 0) std::cout<<kn<<std::endl;
						blender_Obstacles[o]->Neighbors[nn].fn[0] = -kn * en2 * dn;
					}
					else if (delta_dn < 0.0) { // Loading
						// if (Obstacles[o]->group == 0) std::cout<<kn<<std::endl;
						blender_Obstacles[o]->Neighbors[nn].fn[0] += -kn * delta_dn;
					}
					else {
						// if (Obstacles[o]->group == 0) std::cout<<kn<<std::endl;
						blender_Obstacles[o]->Neighbors[nn].fn[0] = -kn * dn; // First time loading
					}
				}

				sumfn += blender_Obstacles[o]->Neighbors[nn].fn[0];
				blender_Obstacles[o]->Neighbors[nn].dn[0] = dn;

				//TODO: ask vincent to give you a hand with this

				// === Friction force
				//vec2r lever = MP[pn].corner[corner] - Obstacles[o]->pos;
                // vec2r zCrossLever(-lever.y, lever.x);
				// double delta_dt = (
				// 	(MP[pn].pos - MP[pn].prev_pos)
				// 	-dt * (Obstacles[o]->vel + Obstacles[o]->vrot * zCrossLever)
				// ) * T;
                //
				/*
				//this is the original part
				vec3r delta_dt = MP[pn].pos - MP[pn].prev_pos;
                double delta_dt1 = delta_dt * T1;
                double delta_dt2 = delta_dt * T2;
				*/

				vec3r lever = MP[pn].pos - blender_Obstacles[o]->pos; //FIXME: Here MPpos should be replaced with corner at some point
				double delta_dt1 = (
					(MP[pn].pos - MP[pn].prev_pos)
					-dt * (blender_Obstacles[o]->vel)) * T1;  //FIXME: add rotational part like in 2d

				double delta_dt2 = (
					(MP[pn].pos - MP[pn].prev_pos)
					-dt * (blender_Obstacles[o]->vel)) * T2;

                //Obstacles[o]->Neighbors[nn].ft += -kt * delta_dt;
                blender_Obstacles[o]->Neighbors[nn].ft[0] += -kt * delta_dt1;
                double threshold = mu * blender_Obstacles[o]->Neighbors[nn].fn[0];


				if (blender_Obstacles[o]->Neighbors[nn].ft[0] > threshold)
					blender_Obstacles[o]->Neighbors[nn].ft[0] = threshold;
				if (blender_Obstacles[o]->Neighbors[nn].ft[0] < -threshold)
					blender_Obstacles[o]->Neighbors[nn].ft[0] = -threshold;

                blender_Obstacles[o]->Neighbors[nn].ft[1] += -kt * delta_dt2;

                if (blender_Obstacles[o]->Neighbors[nn].ft[1] > threshold)
					blender_Obstacles[o]->Neighbors[nn].ft[1] = threshold;
				if (blender_Obstacles[o]->Neighbors[nn].ft[1] < -threshold)
					blender_Obstacles[o]->Neighbors[nn].ft[1] = -threshold;


				// === Resultant force
				vec3r f = blender_Obstacles[o]->Neighbors[nn].fn[0] * N + blender_Obstacles[o]->Neighbors[nn].ft[0] * T1
                          + blender_Obstacles[o]->Neighbors[nn].ft[1] * T2;
				MP[pn].f += f;

                blender_Obstacles[o]->force -= f;

				// === Resultant moment (only on the obstacle)
				blender_Obstacles[o]->mom += norm(cross(lever, -f));
				//FIXME: this is just a weird test. I added norm (use as a vector!)

			}
			else {
                blender_Obstacles[o]->Neighbors[nn].dn[0] = 0.0;
                blender_Obstacles[o]->Neighbors[nn].dt[0] = 0.0;
                blender_Obstacles[o]->Neighbors[nn].fn[0] = 0.0;
                blender_Obstacles[o]->Neighbors[nn].ft[0] = 0.0;
                blender_Obstacles[o]->Neighbors[nn].ft[1] = 0.0;
			} // end if else touch

		} // end for-loop over neighbors nn
	} // end for-loop over obstacles o
}

void MPMbox::boundaryConditions(){
	//new implementation
	double kn, kt, en2, mu;
	size_t g1, g2;

	for (size_t o = 0 ; o < Obstacles.size() ; ++o) {
		for (size_t nn = 0 ; nn < Obstacles[o]->Neighbors.size() ; ++nn) {

			size_t pn = Obstacles[o]->Neighbors[nn].PointNumber;
			double dn = 0.0;
      double sumfn = 0.0;
      if ( Obstacles[o]->touch(MP[pn],0,dn) ) {

				g1 = (size_t)(MP[pn].groupNb);
				g2 = Obstacles[o]->group;
				kn = dataTable.get(id_kn, g1, g2);
				kt = dataTable.get(id_kt, g1, g2);
				en2 = dataTable.get(id_en2, g1, g2);
				mu = dataTable.get(id_mu, g1, g2);

				vec3r N, T1, T2;
				Obstacles[o]->getContactFrame(MP[pn], N, T1, T2);
				//TODO: Update neighbor class like in 2D and update the BC correspondingly (also the obstacles)
				// === Normal force
				if (dn < 0.0) { // overlapping
					double delta_dn = dn - Obstacles[o]->Neighbors[nn].dn[0];
					if (delta_dn > 0.0) {      // Unloading
						// if (Obstacles[o]->group == 0) std::cout<<kn<<std::endl;
						Obstacles[o]->Neighbors[nn].fn[0] = -kn * en2 * dn;
					}
					else if (delta_dn < 0.0) { // Loading
						// if (Obstacles[o]->group == 0) std::cout<<kn<<std::endl;
						Obstacles[o]->Neighbors[nn].fn[0] += -kn * delta_dn;
					}
					else {
						// if (Obstacles[o]->group == 0) std::cout<<kn<<std::endl;
						Obstacles[o]->Neighbors[nn].fn[0] = -kn * dn; // First time loading
					}
				}
        sumfn += Obstacles[o]->Neighbors[nn].fn[0];
				Obstacles[o]->Neighbors[nn].dn[0] = dn;

				//TODO: ask vincent to give you a hand with this

				// === Friction force
				//vec2r lever = MP[pn].corner[corner] - Obstacles[o]->pos;
                // vec2r zCrossLever(-lever.y, lever.x);
				// double delta_dt = (
				// 	(MP[pn].pos - MP[pn].prev_pos)
				// 	-dt * (Obstacles[o]->vel + Obstacles[o]->vrot * zCrossLever)
				// ) * T;
                //
				/*
				//this is the original part
				vec3r delta_dt = MP[pn].pos - MP[pn].prev_pos;
                double delta_dt1 = delta_dt * T1;
                double delta_dt2 = delta_dt * T2;
				*/

				vec3r lever = MP[pn].pos - Obstacles[o]->pos; //FIXME: Here MPpos should be replaced with corner at some point
				double delta_dt1 = (
					(MP[pn].pos - MP[pn].prev_pos)
					-dt * (Obstacles[o]->vel)) * T1;  //FIXME: add rotational part like in 2d

				double delta_dt2 = (
					(MP[pn].pos - MP[pn].prev_pos)
					-dt * (Obstacles[o]->vel)) * T2;

                //Obstacles[o]->Neighbors[nn].ft += -kt * delta_dt;
                Obstacles[o]->Neighbors[nn].ft[0] += -kt * delta_dt1;
                double threshold = mu * Obstacles[o]->Neighbors[nn].fn[0];


				if (Obstacles[o]->Neighbors[nn].ft[0] > threshold)
					Obstacles[o]->Neighbors[nn].ft[0] = threshold;
				if (Obstacles[o]->Neighbors[nn].ft[0] < -threshold)
					Obstacles[o]->Neighbors[nn].ft[0] = -threshold;

                Obstacles[o]->Neighbors[nn].ft[1] += -kt * delta_dt2;

                if (Obstacles[o]->Neighbors[nn].ft[1] > threshold)
					Obstacles[o]->Neighbors[nn].ft[1] = threshold;
				if (Obstacles[o]->Neighbors[nn].ft[1] < -threshold)
					Obstacles[o]->Neighbors[nn].ft[1] = -threshold;


				// === Resultant force
				vec3r f = Obstacles[o]->Neighbors[nn].fn[0] * N + Obstacles[o]->Neighbors[nn].ft[0] * T1
                          + Obstacles[o]->Neighbors[nn].ft[1] * T2;
				MP[pn].f += f;

                Obstacles[o]->force -= f;

				// === Resultant moment (only on the obstacle)
				Obstacles[o]->mom += norm(cross(lever, -f));
				//FIXME: this is just a weird test. I added norm (use as a vector!)

			}
			else {
                Obstacles[o]->Neighbors[nn].dn[0] = 0.0;
                Obstacles[o]->Neighbors[nn].dt[0] = 0.0;
                Obstacles[o]->Neighbors[nn].fn[0] = 0.0;
                Obstacles[o]->Neighbors[nn].ft[0] = 0.0;
                Obstacles[o]->Neighbors[nn].ft[1] = 0.0;
			} // end if else touch

		} // end for-loop over neighbors nn
	} // end for-loop over obstacles o
}

/// @fixme this function is ok only for regular grids
void MPMbox::save_vtk_grid(double x0, double y0, double z0)
{
	char fname[256];
	sprintf (fname, "%s/grid.vtk", result_folder.c_str());
	std::cout << "Save " << fname << "... ";
	std::ofstream file(fname);

	// Header
	file << "# vtk DataFile Version 2.0" << std::endl;
	file << "The grid" << std::endl;
	file << "ASCII" << std::endl;

	/*//new part!!!
	file << "DATASET POLYDATA" << std::endl;  //should be unstructured grid instead of polydata but the former should be the correct
	file << "POINTS " << nodes.size() << " float" << std::endl;
	for (size_t i = 0 ; i < nodes.size() ; ++i) file << nodes[i].pos << " 0" << std::endl;


	file << " " << std::endl;
	file << "POLYGONS " << Elem.size() << " " << (Elem.size())*5 << std::endl;
	for (size_t i = 0 ; i < Elem.size() ; ++i)   {
	file<< "4 " << Elem[i].I[0] << " " << Elem[i].I[1] << " " << Elem[i].I[3] << " " << Elem[i].I[2] << std::endl;
	}
	end of new part
	*/

	//old part
	file << "DATASET RECTILINEAR_GRID" << std::endl;
	file << "DIMENSIONS " << Grid.Nx+1 << " " << Grid.Ny+1 << " " << Grid.Nz+1<< std::endl;

	file << "X_COORDINATES " << Grid.Nx+1 << " float" << std::endl;
	for (int i = 0 ; i <= Grid.Nx ; ++i) file << x0 + i*Grid.lx << " ";
	file << std::endl;

	file << "Y_COORDINATES " << Grid.Ny+1 << " float" << std::endl;
	for (int i = 0 ; i <= Grid.Ny ; ++i) file << y0 + i*Grid.ly << " ";
	file << std::endl;

	file << "Z_COORDINATES " << Grid.Nz+1 << " float" << std::endl;
	for (int i = 0 ; i <= Grid.Nz ; ++i) file << z0 + i*Grid.lz << " ";
	file << std::endl;

	//till here
	std::cout << "done." << std::endl;
}

void MPMbox::save_vtk(const char* base, int num)
{
	///disabled while coding
	/*

	// Open file
	char name[256];
	sprintf(name,"%s%d.vtk",base,num);
	std::cout << "Save " << name << "... ";
	std::ofstream file(name);

	// Header
	file << "# vtk DataFile Version 2.0" << std::endl;
	file << "State at time " << t << std::endl;
	file << "ASCII" << std::endl;

	file << "DATASET POLYDATA" << std::endl;
	file << "POINTS " << MP.size()*4 << " float" << std::endl;
	for (size_t p = 0 ; p < MP.size() ; ++p)   {
		for (int j = 0 ; j <= 3 ; ++j) {
			file << MP[p].corner[j]  << " 0" << std::endl;
		}
	}

	file << " " << std::endl;
	file << "POLYGONS " << MP.size() << " " << (MP.size())*5 << std::endl;
	for (size_t i = 0 ; i < MP.size() ; ++i)   {
	file<< "4 " << 0+4*i << " " << 1+4*i << " " << 3+4*i << " " << 2+4*i << std::endl;
	}
	*/

	// Preparation for smoothed data
	int *I;
	for (size_t p = 0 ; p < MP.size() ; p++) {
		shapeFunction->computeInterpolationValues(*this, p);
	}

	// Reset nodal mass
	for (size_t n = 0 ; n < liveNodeNum.size() ; n++) {
		nodes[liveNodeNum[n]].mass = 0.0;
		nodes[liveNodeNum[n]].vel.reset();
		nodes[liveNodeNum[n]].stress.xx = nodes[liveNodeNum[n]].stress.xy = nodes[liveNodeNum[n]].stress.xz
                                        = nodes[liveNodeNum[n]].stress.yx = nodes[liveNodeNum[n]].stress.yy
                                        = nodes[liveNodeNum[n]].stress.yz = nodes[liveNodeNum[n]].stress.zx
                                        = nodes[liveNodeNum[n]].stress.zy = nodes[liveNodeNum[n]].stress.zz = 0.0;
	}

	// Nodal mass
	for (size_t p = 0 ; p < MP.size() ; p++) {
		I = &(Elem[MP[p].e].I[0]);
		for (int r = 0 ; r < 8 ; r++) {
			nodes[I[r]].mass += MP[p].N[r] * MP[p].mass;
		}
	}

	///show material points here!!!!!!!!!!!!!!!!!! 29-12-2014
	// Open file
	//TODO: See what sort of things I output in 2D
	char name[256];
	sprintf(name,"%s/%s%d.vtk",result_folder.c_str(), base, num);
	std::cout << "Save " << name << " #MP: " << MP.size() << " Time: " << t << " ... ";
	std::ofstream file(name);

	// Header
	file << "# vtk DataFile Version 2.0" << std::endl;
	file << "State at time " << t << std::endl;
	file << "ASCII" << std::endl;

	if (actualMPflag == false) {

		file << "DATASET UNSTRUCTURED_GRID" << std::endl;
		file << "POINTS " << MP.size()*8 << " float" << std::endl;  //because it has 8 corners
		for (size_t p = 0 ; p < MP.size() ; ++p)   {
			for (int j = 0 ; j < 8 ; ++j) {
				file << MP[p].corner[j] << std::endl;
			}
		}

		file << " " << std::endl;
		file << "CELLS " << MP.size() << " " << (MP.size())*9 << std::endl;
		for (size_t i = 0 ; i < MP.size() ; ++i)   {
		file<< "8 " << 0+8*i << " " << 1+8*i << " " << 5+8*i << " " << 4+8*i << " " << 2+8*i << " " << 3+8*i << " " << 7+8*i << " " << 6+8*i << std::endl;
		}

		/// CELL TYPES ==========
		file << std::endl << "CELL_TYPES " << MP.size() << std::endl;
		for (size_t i = 0 ; i < MP.size() ; ++i)   {
			file<<"12"<<std::endl;
		}
	}

	else {
		file << "DATASET POLYDATA" << std::endl;
		file << "POINTS " << MP.size() << " float" << std::endl;
		for (size_t p = 0 ; p < MP.size() ; ++p)   {
			file << MP[p].pos  << std::endl;
		}
		/// POINT DATA ==========
		file << std::endl << "POINT_DATA " << MP.size() << std::endl;
	}

	// Size of material points (used for display with glyph)  ---------------
	file << std::endl << "VECTORS radius float" << std::endl;
	for (size_t i = 0 ; i < MP.size() ; ++i) {
		file << std::pow(MP[i].vol, 1/3.) << " 0 0" << std::endl;
	}
	/*
	file << std::endl << "VECTORS forcesContact float " << std::endl;
	for (size_t i = 0 ; i < MP.size() ; ++i) {
		file << MP[i].f<< std::endl;
	}
	*/
	file << std::endl << "VECTORS velocity float " << std::endl;
	for (size_t i = 0 ; i < MP.size() ; ++i) {
		file << MP[i].vel<< std::endl;
	}

	// Raw total Stress  ---------------
	file << std::endl << "TENSORS raw_totalStress float" << std::endl;
	for (size_t i = 0 ; i < MP.size() ; ++i) {
		file << MP[i].stress.xx << " " << MP[i].stress.xy << " " << MP[i].stress.xz << std::endl;
		file << MP[i].stress.yx << " " << MP[i].stress.yy << " " << MP[i].stress.yz<< std::endl;
		file << MP[i].stress.zx << " " << MP[i].stress.zy << " " << MP[i].stress.zz<< std::endl;
	}

	// Smoothed total Stress  ---------------
	// Nodal Stress
	for (size_t p = 0 ; p < MP.size() ; p++) {
		I = &(Elem[MP[p].e].I[0]);
		for (int r = 0 ; r < 8 ; r++) {
			nodes[I[r]].stress += MP[p].N[r] * MP[p].mass * MP[p].stress / nodes[I[r]].mass;
		}
	}

	// Material-point stress (Smoothed)
	std::vector<mat9r> smoothStress(MP.size());
	for (size_t p = 0 ; p < MP.size() ; p++) {
		I = &(Elem[MP[p].e].I[0]);
		for (int r = 0 ; r < 8 ; r++) {
			smoothStress[p] += nodes[I[r]].stress * MP[p].N[r];
		}
	}
	file << std::endl << "TENSORS smoothed_totalStress float" << std::endl;
	for (size_t i = 0 ; i < MP.size() ; ++i) {
		file << smoothStress[i].xx << " " << smoothStress[i].xy << " " << smoothStress[i].xz << std::endl;
		file << smoothStress[i].yx << " " << smoothStress[i].yy << " " << smoothStress[i].yz << std::endl;
		file << smoothStress[i].zx << " " << smoothStress[i].zy << " " << smoothStress[i].zz << std::endl;
	}

	//plastic stress norm
	file << std::endl << "SCALARS plasticStressNorm float 1" << std::endl;
	file << "LOOKUP_TABLE default" << std::endl;
	//matrix norm (frobenius norm) https://en.wikipedia.org/wiki/Matrix_norm
	for (size_t i = 0 ; i < MP.size() ; ++i) {

		//mat9r temp = MP[i].plasticStress * MP[i].plasticStress.transpose1();
		mat9r temp = MP[i].plasticStress * MP[i].plasticStress.transposed();
		file << sqrt(temp.xx + temp.yy + temp.zz)  << std::endl;
	}


	/*
	//Total displacement
	file << std::endl << "VECTORS totalDisplacement float "<< std::endl;
	for (size_t i = 0 ; i < MP.size() ; ++i) {
		file << MP[i].total_displacement<< std::endl;
	}

    //instant displacement
	file << std::endl << "VECTORS instantDisplacement float "<< std::endl;
	for (size_t i = 0 ; i < MP.size() ; ++i) {
		file << MP[i].instant_displacement<< std::endl;
	}
	*/
	std::cout << "done." << std::endl;
}

void MPMbox::save_MPpos(const char* base, int num) {
	char name[256];
	sprintf(name,"%s/%s%d.txt",result_folder.c_str(), base, num);
	// std::cout << "Save " << name << " #MP: " << MP.size() << " Time: " << t << " ... ";
	std::ofstream file(name);
	// file << MP.size() << '\n';
	// file << "comment" << '\n';
	for (size_t p = 0; p < MP.size(); p++){
		file <</*"C " << */MP[p].pos<<std::endl;
	}
}



void MPMbox::save_vtk_obst(const char* base, int numb, int obstacleNb)
{

	std::vector<int> nbNodesOfObstacle;
	std::vector<vec3r> coords;

	// for (size_t o = 0 ; o < Obstacles.size() ; o++) {
	// 	Obstacles[o]->addVtkPoints(coords, nbNodesOfObstacle);
	// 	//int nb = Obstacles[o]->addVtkPoints(coords);
	// 	//std::cout<<"pos of obstacle "<<std::endl<<Obstacles[o]->pos<<std::endl;
	// 	//nbNodesOfObstacle.push_back(nb);  //doing it for each individual set. not suited for leftbiplanar
	// }
	for (size_t o = 0 ; o < Obstacles.size() ; o++) {
		if (Obstacles[o]->group == obstacleNb) {
			Obstacles[o]->addVtkPoints(coords, nbNodesOfObstacle);
		}
	}

	for (size_t o = 0 ; o < blender_Obstacles.size() ; o++) {
		if (blender_Obstacles[o]->group == obstacleNb) {
			blender_Obstacles[o]->addVtkPoints(coords, nbNodesOfObstacle);
		}
	}

	// Open file
	char name[256];

	//sprintf (name, "%s/obstacles.vtk", result_folder.c_str());
	sprintf (name, "%s/group%d%s%d.vtk", result_folder.c_str(), obstacleNb, base, numb);
	std::ofstream file(name);
	// Header
	file << "# vtk DataFile Version 2.0" << std::endl;
	file << "State at time " << t << std::endl;
	file << "ASCII" << std::endl;

	file << "DATASET UNSTRUCTURED_GRID" << std::endl;
	file << "POINTS " << coords.size() << " float" << std::endl;
	for (size_t i = 0 ; i < coords.size() ; ++i)   {
		file << coords[i]  << std::endl;
	}

	file << " " << std::endl;
	int nbData = 0; //necessary for the vtk file
	for (size_t n = 0 ; n < nbNodesOfObstacle.size() ; n++) {
		nbData += nbNodesOfObstacle[n];
	}
	nbData += nbNodesOfObstacle.size();

	file << "CELLS " << nbNodesOfObstacle.size() << " " << nbData << std::endl;
	int num = 0;
	for (size_t i = 0 ; i < nbNodesOfObstacle.size() ; ++i) {
		file << nbNodesOfObstacle[i];
		for (int nn = 0 ; nn < nbNodesOfObstacle[i] ; nn++) {
			file << " " << num++;
		}
		file << std::endl;
	}

	/// CELL TYPES ==========
	file << std::endl << "CELL_TYPES " << nbNodesOfObstacle.size() << std::endl;
	for (size_t i = 0 ; i < nbNodesOfObstacle.size() ; ++i)   {
		if (nbNodesOfObstacle[i] == 4) file << "7" << std::endl; //7 means polygon, 4 means polyline (for more info check vtk documentation)
		else file << "7" << std::endl;
	}

	//std::cout << "done." << std::endl;
}

/// @fixme move this function to the structure 'grid'
void MPMbox::set_grid(int nbElemX, int nbElemY, int nbElemZ, double lx, double ly, double lz)
{
	unitNormal.x = 0;
	unitNormal.y = 1;
	unitNormal.z = 0;
	unitTang.x = 1;
	unitTang.y = 0;
	unitTang.z = 0;

	Grid.Nx = nbElemX;
	Grid.Ny = nbElemY;
	Grid.Nz = nbElemZ;
	Grid.lx = lx;
	Grid.ly = ly;
	Grid.lz = lz;

	// Create the nodes and set their positions
	if (!nodes.empty()) nodes.clear();
	node N;
	N.mass = 0.0;
	N.q.reset();
	N.f.reset();
	N.fb.reset();
	N.xfixed = false;
	N.yfixed = false;
	N.zfixed = false;
	int counter = 0;
	for(int k = 0; k<=Grid.Nz; k++) {
		for (int j = 0 ; j <= Grid.Ny ; j++) {
			for (int i = 0 ; i <= Grid.Nx ; i++) {
                N.number = counter;
				N.pos.x = i * Grid.lx;
				N.pos.y = j * Grid.ly;
				N.pos.z = k * Grid.lz;
                counter++;
				nodes.push_back(N);
			}
		}
	}
	// QUA4 elements
	// 2 _ 3
	// |   |
	// 0 _ 1
	if (!Elem.empty()) Elem.clear();
	element E;
	for (int k = 0;k <Grid.Nz; k++) {
		for (int j = 0 ; j < Grid.Ny ; j++) {
			for (int i = 0 ; i < Grid.Nx ; i++) {
				E.I[0] = (Grid.Nx + 1)*(Grid.Ny + 1)*k + (Grid.Nx + 1) * j + i;
				E.I[1] = (Grid.Nx + 1)*(Grid.Ny + 1)*k + (Grid.Nx + 1) * j + i + 1;
				E.I[2] = (Grid.Nx + 1)*(Grid.Ny + 1)*k + (Grid.Nx + 1) * (j + 1) + i;
				E.I[3] = (Grid.Nx + 1)*(Grid.Ny + 1)*k + (Grid.Nx + 1) * (j + 1) + i + 1;
				E.I[4] = (Grid.Nx + 1)*(Grid.Ny + 1)*(k + 1) + (Grid.Nx + 1) * j + i;
				E.I[5] = (Grid.Nx + 1)*(Grid.Ny + 1)*(k + 1) + (Grid.Nx + 1) * j + i + 1;
				E.I[6] = (Grid.Nx + 1)*(Grid.Ny + 1)*(k + 1) + (Grid.Nx + 1) * (j + 1) + i;
				E.I[7] = (Grid.Nx + 1)*(Grid.Ny + 1)*(k + 1) + (Grid.Nx + 1) * (j + 1) + i + 1;
				Elem.push_back(E);

			}
		}
	}
	//std::cout<<"ELements DEFINED"<<std::endl;
	//std::cin>>delVar;
	//TODO. Set number correct nodes in element for the appropriate Shape function

    for (size_t i = 0; i < nodes.size(); i++) {
        liveNodeNum.push_back(i);
    }
}

void MPMbox::set_BC_line(int line_num, int column0, int column1, int depth0, int depth1, bool xfixed, bool yfixed)
{
	if (nodes.empty()) {
		std::cerr << "Cannot set BC. Grid not yet defined!" << std::endl;
	}

	/*node * N;
	for (int i = column0 ; i <= column1 ; i++) {
		N = &(nodes[line_num * (Grid.Nx + 1) + i]);
		N->xfixed = xfixed;
		N->yfixed = yfixed;
	}
	*/
	node * N;
	for(int k = depth0 ; k<=depth1 ; k++) {
		for (int i = column0 ; i <= column1 ; i++) {
			N = &(nodes[(Grid.Nx + 1) * (Grid.Ny + 1) * k + line_num * (Grid.Nx + 1) + i]);
			N->xfixed = xfixed;
			N->yfixed = yfixed;
		}
	}
}

void MPMbox::set_BC_column(int column_num, int line0, int line1, int depth0, int depth1, bool xfixed, bool yfixed)
{
	if (nodes.empty()) {
		std::cerr << "Cannot set BC. Grid not yet defined!" << std::endl;
	}
	//int delvar;
	node * N;
	for(int k = depth0 ; k<=depth1 ; k++) {
		for (int j = line0 ; j <= line1 ; j++) { //HAS TO BE FIXED FOR IRREGULAR GRIDS
			N = &(nodes[(Grid.Nx + 1) * (Grid.Ny + 1) * k + j * (Grid.Nx + 1) + column_num]);
			N->xfixed = xfixed;
			N->yfixed = yfixed;
			//std::cout<<(Grid.Nx + 1) * (Grid.Ny + 1) * k + j * (Grid.Nx + 1) + column_num<<std::endl;
			//std::cin>>delvar;
		}
	}
}

void MPMbox::move_MP(double x0, double y0, double z0, double dx, double dy,double dz, double thetaZ, double thetaY)
{
	thetaZ = thetaZ * Mth::pi/180;
	thetaY = thetaY * Mth::pi/180;
	mat9r rotZ(cos(thetaZ), -sin(thetaZ), 0, sin(thetaZ), cos(thetaZ), 0, 0, 0, 1);
	mat9r rotY(cos(thetaY), 0, sin(thetaY), 0, 1, 0, -sin(thetaY), 0, cos(thetaY));

	mat9r rotationMat = rotZ*rotY;  //you can easily add rotation around the x axis

	for (size_t p = 0 ; p < MP.size() ; p++) {

		MP[p].corner[0].x = - halfSizeMP;
		MP[p].corner[0].y = - halfSizeMP;
		MP[p].corner[0].z = - halfSizeMP;
		MP[p].corner[1].x = + halfSizeMP;
		MP[p].corner[1].y = - halfSizeMP;
		MP[p].corner[1].z = - halfSizeMP;
		MP[p].corner[2].x = - halfSizeMP;
		MP[p].corner[2].y = + halfSizeMP;
		MP[p].corner[2].z = - halfSizeMP;
		MP[p].corner[3].x = + halfSizeMP;
		MP[p].corner[3].y = + halfSizeMP;
		MP[p].corner[3].z = - halfSizeMP;
		MP[p].corner[4].x = - halfSizeMP;
		MP[p].corner[4].y = - halfSizeMP;
		MP[p].corner[4].z = + halfSizeMP;
		MP[p].corner[5].x = + halfSizeMP;
		MP[p].corner[5].y = - halfSizeMP;
		MP[p].corner[5].z = + halfSizeMP;
		MP[p].corner[6].x = - halfSizeMP;
		MP[p].corner[6].y = + halfSizeMP;
		MP[p].corner[6].z = + halfSizeMP;
		MP[p].corner[7].x = + halfSizeMP;
		MP[p].corner[7].y = + halfSizeMP;
		MP[p].corner[7].z = + halfSizeMP;
	}

	double newx, newy, newz;
	for (size_t p = 0 ; p < MP.size() ; p++) {
		/*
		MP[p].F.xx = Rxx; MP[p].F.xy = Rxy;
		MP[p].F.yx = Ryx; MP[p].F.yy = Ryy;
		*/

		newx = x0 + (MP[p].pos.x-x0)*rotationMat.xx + (MP[p].pos.y-y0)*rotationMat.xy + (MP[p].pos.z-z0)*rotationMat.xz + dx;
		newy = y0 + (MP[p].pos.x-x0)*rotationMat.yx + (MP[p].pos.y-y0)*rotationMat.yy + (MP[p].pos.z-z0)*rotationMat.yz + dy;
		newz = z0 + (MP[p].pos.x-x0)*rotationMat.zx + (MP[p].pos.y-y0)*rotationMat.zy + (MP[p].pos.z-z0)*rotationMat.zz + dz;
		MP[p].pos.x = newx;
		MP[p].pos.y = newy;
		MP[p].pos.z = newz;
		/*
		for (int c = 0 ; c < 4 ; c++) {

			newx = MP[p].pos.x + (MP[p].corner[c].x)*Rxx + (MP[p].corner[c].y)*Rxy;
			newy = MP[p].pos.y + (MP[p].corner[c].x)*Ryx + (MP[p].corner[c].y)*Ryy;
			MP[p].corner[c].x = newx;
			MP[p].corner[c].y = newy;
		}
		*/
	}

	//std::cout<<"initial position corner 0"<<std::endl;
	//std::cout<<MP[0].corner[0]<<std::endl;
	//std::cin>>newx;
}


//
// void MPMbox::set_properties(double mu1, double K1, double e21, double Kt1)
// {
//
// 	mu = mu1;
// 	K = K1;
// 	e2 = e21;
// 	Kt = Kt1;
// }


void MPMbox::set_K0_stress(double nu, double rho0)
{
	if (MP.empty()) return;
	vec3r ug = gravity;
	double g = ug.normalize();

	vec3r surfacePoint = MP[0].pos;
	double dmin = MP[0].pos * ug;
	double d = dmin;
	for (size_t p = 1 ; p < MP.size() ; ++p) {
		d = MP[p].pos * ug;
		if (d < dmin) {
			dmin = d;
			surfacePoint = MP[p].pos;
		}
	}

	for (size_t p = 0 ; p < MP.size() ; ++p) {
		d = (MP[p].pos - surfacePoint) * ug;
		MP[p].stress.yy = -d * g * rho0;
		MP[p].stress.xx = -d * g * rho0 * nu / (1 - nu);
		MP[p].stress.xy = MP[p].stress.yx = 0.0;
	}

}
