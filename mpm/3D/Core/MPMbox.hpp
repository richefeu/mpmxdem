#ifndef MPMBOX_HPP_63DBDC7C
#define MPMBOX_HPP_63DBDC7C

#include <cstddef>
#include <vector>
#include <string>
#include <map>
#include <set>
#include <fstream>
#include <ctime>
#include <exception>
#include <algorithm>

//TODO: see how in 2D all these files are called directly without '../'
#include "Mth.hpp"
#include "vec2.hpp"
#include "vec3.hpp"
#include "mat4.hpp"
#include "mat9.hpp"
#include <fileTool.hpp>
#include <factory.hpp>
#include <DataTable.hpp>
#include "Grid.hpp"
#include "Node.hpp"
#include "Element.hpp"

#include "MaterialPoint.hpp"
#include <Obstacles/Obstacle.hpp>
#include <blender_Obstacles/blender_Obstacle.hpp>

#include "Neighbor.hpp"

struct ShapeFunction;
struct OneStep;

struct MPMbox
{
	std::vector<node>           nodes;  // The nodes of the Eulerian grid
    std::vector<int>           liveNodeNum;   // list of nodes being used during each time step
	std::vector<element>        Elem;   // QUA4 elements of the grid
	std::vector<MaterialPoint>  MP;     // Material Points

	std::ofstream logFile;
	//std::ofstream logFile1;

	std::vector<Obstacle*> 		Obstacles; // List of rigid obstacles
	std::vector<blender_Obstacle*> blender_Obstacles;

	std::string result_folder;
	std::string oneStepType;

	ShapeFunction * shapeFunction;
	OneStep * oneStep;  //Type of routine to be used
	std::map<std::string,ConstitutiveModel*> models;

	bool planeStrain; 	// Plane strain assumption (plane stress if false, default)
	//double lgrid;     // Size of QUA4 elements (mesh refinement is not possible)
	//int Nx,Ny;        // Number of grid-elements (QUA4) along x and y directions
	grid Grid;
	double tolmass;   	// Tolerance for the mass of a MaterialPoint
	vec3r gravity;     	// The gravity acceleration vector

	int step;
	int nstep;        	// Number of steps to be done
	double finalTime;   //time in seconds at which the simulation ends (to replace nstep)
	int vtkPeriod;   	// Number of steps between vtk files
	int proxPeriod;    	// Number of steps between proximity check (rebuild the neighbor list)
	double dt;        	// Time increment
	double t;         	// Current time
	double sizeMP; 	  	// (size of the volume for the MP) has to be erased any time soon!!!!!!!!!!
	double halfSizeMP;	// half sizeMP, (for convenience)
	double securDistFactor;		// Homothetic factor of shapes for proximity tests
	std::set<int> obstacleGroups;  //a set used to save obstacle in different files

	DataTable dataTable;
	size_t id_mu, id_kn, id_en2, id_kt;

	//double mu, K, e2, Kt;
	double theta;
	bool actualMPflag;

	vec3r unitNormal;  	//test that works only for the bottom boundary condition!!!!!!!!!!!
	vec3r unitTang;

	size_t number_MP; 			//used to check proximity if # of MP has changed (some "unknown" points could enter the obstacle and suddenly be detected once they are way inside)

	MPMbox();
	~MPMbox();
	void showAppBanner();
	void read(const char * name);
	void MPinGridCheck();
	void cflCondition();
	void checkProximity();

	void save_vtk_grid(double x0 = 0.0, double y0 = 0.0, double z0 = 0.0);
	void save_vtk_obst(const char* base, int numb, int obstacleNb);
	void save_vtk(const char* base, int num);
	void save_MPpos(const char* base, int num);
	void init(const char * name);
	void run();

	//Functions used in OneStep
	void boundaryConditions();
	void blenderBoundaryConditions();

	// Preprocessing routines

	void set_grid(int nbElemX, int nbElemY, int nbElemZ, double lx, double ly, double lz);
	void set_BC_line(int line_num, int column0, int column1, int depth0, int depth1, bool xfixed, bool yfixed);
	void set_BC_column(int column_num, int line0, int line1, int depth0, int depth1, bool xfixed, bool yfixed);
	void move_MP(double x0, double y0, double z0, double dx, double dy,double dz, double thetaZ, double thetaY);
	void set_MP_grid(int group, ConstitutiveModel * CM, double rho, double x0, double y0, double z0, double x1, double y1, double z1, double size);
	void set_K0_stress(double nu, double rho0);
	// void set_properties(double mu1, double K1, double e21, double Kt1);

};

#endif /* end of include guard: MPMBOX_HPP_63DBDC7C */
