#ifndef MATERIALPOINT_HPP_0FF29780
#define MATERIALPOINT_HPP_0FF29780


#include "vec2.hpp"
#include "vec3.hpp"
#include "mat4.hpp"
#include "mat9.hpp"


struct ConstitutiveModel;

struct MaterialPoint
{
	int groupNb;        	    // Group Number
	double mass;                // Mass
	double size;        		// size of the sides of squared MP
	double vol;                 // Volume
	double density;				// Density
	vec3r pos;                   // Position
    vec3r prev_pos;              // Previous position
	vec3r vel;                   // Velocity
	vec3r smoothVel;
	double securDist;   		// Security distance for contact detection

	vec3r f;                     // Force
	vec3r q;                     // Momentum (mass times velocity)
	mat9r strain;                // Total strain
	mat9r stress;                // Total stress
	mat9r prevStress;
	mat9r plasticStrain;         // Plastic Strain
	mat9r plasticStress;         // Plastic Stress
	mat9r deltaStrain;
	double yieldFunctionUncorrected;
	double yieldFunctionCorrected;
	double PlasticYieldStress;  // Plastic Yield Stress
	double N[8];                // Value of shape function according to the position of the Material Point
	vec3r gradN[8];              // Gradient of shape function according to the position of the Material Point
	int e;                      // Identify the element to which the point belongs
	mat9r F;
	mat9r dFp;
	vec3r corner[8];
	int splitCount;

	double dt;


	double prevDist[4];
	int gridMP;
	vec2r P[4];					// corners that define a material point
	int distance[4];			//distance from corner of grid to corners that define material point
	double prevDistance[4];		//this if for friction (the prevDist that is used for the normal force must be erased)
    vec3r total_displacement;
    vec3r instant_displacement;

	ConstitutiveModel * constitutiveModel; // Pointer to the constitutive model

	MaterialPoint ();
};

#endif /* end of include guard: MATERIALPOINT_HPP_0FF29780 */
