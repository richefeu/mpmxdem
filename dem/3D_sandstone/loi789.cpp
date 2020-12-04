/*
tk: Modifier pour respecter le format dans Lagamine
    Routine pour loi 3D ELASTIC ISO (équivalent à ELA3D dans Lagamine)

Pour compiler, linker et tester :

gfortran -c testF.f
g++ -c LHN_Hook.cpp  -I
gfortran -o CF testF.o LHN_Hook_3D.o -lstdc++
./CF

20160421 - Nettoyage du code loi789.cpp
*/

#include "PBC3D.hpp"

// Pour que le prototype de fonction dans le .o soit utilisable par fortran 
extern "C" {
        void loi789_(double QA[], double QB[], double dFmoinsI[3][3], double SIG[3][3], double PARAM[], double *I, double STC[4][4]);
}

// - Rq 1 : lorsqu'on passe un tableau en C on passe en réalité un pointer vers le premier élément du tableau
// donc c'est compatible avec les fonctions fortran.
// - Rq 2 : je n'ai pas mis '3D' dans le nom de la fonction sinon ça link pas
// - Utilisation en fortran : lhn_hook (...)
//
// F (input) est le tenseur de transformation
// Sigma (output) est le tenseur de contrainte
// QA (input) le vecteur (notation ingénieur) de déformation actuelle
// QB (output) le vecteur (notation ingénieur) de deformation après transformation par F 
// Param (intput) contient le module d'Young suivit du coefficient de Poisson
void loi789_(double QA[], double QB[], double dFmoinsI[3][3], double SIG[3][3], double PARAM[], double *I, double STC[4][4])
{
	const double tolSigma = 1.0e-2;
	const int nstepConv = 10;
	const int nstepMax = 2000;
		
	PBC3Dbox box;

	// Transfer the parameters to 'box'
	box.kn        = PARAM[0];
	box.kt        = PARAM[1];
	box.mu        = PARAM[2];
	box.density   = PARAM[3];
	box.dampRate  = PARAM[4];
	box.Cell.mass = PARAM[5];
	box.fcoh      = PARAM[6];
	
	box.initLagamine(QA);
	box.transform_and_hold(dFmoinsI, I, &tolSigma, &nstepConv, nstepMax);
	box.endLagamine(QB, SIG);

	double L[9][9];
	box.getOperatorKruyt2(L);

	// Recuperation C Lagamine a partir de Kruyt
	STC[0][0] = L[0][0]; 	
	STC[0][1] = L[0][3]; 
	STC[0][2] = L[0][6]; 
	STC[0][3] = L[0][1]; 

	STC[1][0] = L[3][0]; 	
	STC[1][1] = L[3][3]; 
	STC[1][2] = L[3][6]; 
	STC[1][3] = L[3][1]; 

	STC[2][0] = L[6][0]; 	
	STC[2][1] = L[6][3];
	STC[2][2] = L[6][6]; 
	STC[2][3] = L[6][1]; 

	STC[3][0] = L[1][0]; 	
	STC[3][1] = L[1][3]; 
	STC[3][2] = L[1][6]; 
	STC[3][3] = L[1][1]; 
}
