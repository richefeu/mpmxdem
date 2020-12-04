#include "set_MP_grid.hpp"
#include "../Core/MPMbox.hpp"
// #include <Core/MaterialPoint.hpp>
#include "../ConstitutiveModels/ConstitutiveModel.hpp"

#include <factory.hpp>
static Registrar<Command, set_MP_grid> registrar("set_MP_grid");

void set_MP_grid::read(std::istream & is)
{
	is >> groupNb >> modelName >> rho >> x0 >> y0 >> z0 >> x1 >> y1 >> z1 >> size;
}

void set_MP_grid::exec()
{
	/*
	if (box->Grid.lx / size > 3.0 || box->Grid.ly / size > 3.0) {
		std::cerr << "@set_MP_grid::exec, Check Grid size - MP size ratio" << std::endl;
		exit(0);
	}
	*/
	auto itCM = box->models.find(modelName);
	if (itCM == box->models.end()) {
		std::cerr << "@set_MP_grid::exec, model " << modelName << " not found" << std::endl;
	}
	ConstitutiveModel * CM = itCM->second;

	MaterialPoint P;
	P.groupNb = groupNb;
	P.vol = size * size * size;
	P.density = rho;
	P.mass = P.vol * rho;
	P.vel.reset();
	P.size = size;


	P.stress.xx = P.stress.xy = P.stress.xz = P.stress.yx = P.stress.yy = P.stress.yz = P.stress.zx = P.stress.zy = P.stress.zz = 0.0;
	P.strain.xx = P.strain.xy = P.strain.xz = P.strain.yx = P.strain.yy = P.strain.yz = P.strain.zx = P.strain.zy = P.strain.zz = 0.0;
	P.plasticStress.xx = P.plasticStress.xy = P.plasticStress.xz = P.plasticStress.yx = P.plasticStress.yy = P.plasticStress.yz = P.plasticStress.zx = P.plasticStress.zy = P.plasticStress.zz = 0.0;
	P.plasticStrain.xx = P.plasticStrain.xy = P.plasticStrain.xz = P.plasticStrain.yx = P.plasticStrain.yy = P.plasticStrain.yz = P.plasticStrain.zx = P.plasticStrain.zy = P.plasticStrain.zz = 0.0;

	P.constitutiveModel = CM;
	P.q = P.mass * P.vel; ///change
	P.splitCount = 0;
	vec2r r1;

	// FIXME: what are those normal and tang needed for? (16/05/18)
	vec2r normal;
	vec2r tang;

	normal.x = 0;
	normal.y = 1;
	tang.x = 1;
	tang.y = 0;


	//new loop (16/05/18)
	double halfSizeMP = 0.5 * size;
	double nbMPX = (x1 - x0)/size;
	double nbMPY = (y1 - y0)/size;
	double nbMPZ = (z1 - z0)/size;
	nbMPX = (int)nbMPX;
	nbMPY = (int)nbMPY;
	nbMPZ = (int)nbMPZ;

	for (int k = 0; k < nbMPZ; k++){
		for (int i = 0; i < nbMPY; i ++) {
			for (int j = 0; j < nbMPX; j ++){
				P.pos.set(x0 + halfSizeMP + size * j, y0 + halfSizeMP + size * i, z0 + halfSizeMP + size * k);
				// P.nb = counter;
				// counter++;
				box->MP.push_back(P);
			}
		}
	}


	// //OLD LOOP
	// double half = 0.5 * size;
	// for (double z = z0 + half; z <= z1 - half; z +=size) {
	// 	for (double y = y0 + half ; y <= y1 - half ; y += size) {
	// 		for (double x = x0 + half ; x <= x1 - half ; x += size) {
	// 			P.pos.x = x;
	// 			P.pos.y = y;
	// 			P.pos.z = z;
	// 			//std::cout<<"P.pos "<<P.pos<<std::endl;
	// 			CM->initializeMaterialPoint(P); /// check if this is a problem
	// 			box->MP.push_back(P);
	// 		}
	// 	}
	// }

	for (size_t p = 0 ; p < box->MP.size() ; p++) {  //THIS MUST BE MODIFIED TAKING INTO ACCOUNT THAT IT IS NOT ALWAYS HORIZONTAL
		//Position of corners
		// 2 _ 3
		// |   |
		// 0 _ 1

		box->MP[p].corner[0].x = box->MP[p].pos.x - 0.5*size;
		box->MP[p].corner[0].y = box->MP[p].pos.y - 0.5*size;
		box->MP[p].corner[0].z = box->MP[p].pos.z - 0.5*size;
		box->MP[p].corner[1].x = box->MP[p].pos.x + 0.5*size;
		box->MP[p].corner[1].y = box->MP[p].pos.y - 0.5*size;
		box->MP[p].corner[1].z = box->MP[p].pos.z - 0.5*size;
		box->MP[p].corner[2].x = box->MP[p].pos.x - 0.5*size;
		box->MP[p].corner[2].y = box->MP[p].pos.y + 0.5*size;
		box->MP[p].corner[2].z = box->MP[p].pos.z - 0.5*size;
		box->MP[p].corner[3].x = box->MP[p].pos.x + 0.5*size;
		box->MP[p].corner[3].y = box->MP[p].pos.y + 0.5*size;
		box->MP[p].corner[3].z = box->MP[p].pos.z - 0.5*size;
		box->MP[p].corner[4].x = box->MP[p].pos.x - 0.5*size;
		box->MP[p].corner[4].y = box->MP[p].pos.y - 0.5*size;
		box->MP[p].corner[4].z = box->MP[p].pos.z + 0.5*size;
		box->MP[p].corner[5].x = box->MP[p].pos.x + 0.5*size;
		box->MP[p].corner[5].y = box->MP[p].pos.y - 0.5*size;
		box->MP[p].corner[5].z = box->MP[p].pos.z + 0.5*size;
		box->MP[p].corner[6].x = box->MP[p].pos.x - 0.5*size;
		box->MP[p].corner[6].y = box->MP[p].pos.y + 0.5*size;
		box->MP[p].corner[6].z = box->MP[p].pos.z + 0.5*size;
		box->MP[p].corner[7].x = box->MP[p].pos.x + 0.5*size;
		box->MP[p].corner[7].y = box->MP[p].pos.y + 0.5*size;
		box->MP[p].corner[7].z = box->MP[p].pos.z + 0.5*size;

	}
	std::cout<<"@set_MP_grid::exec, total number of MPs: "<<box->MP.size()<<std::endl;


	/*





	double halfSizeMP = 0.5 * size;

	MaterialPoint P(groupNb, size, rho, CM);
	int counter = 0;
	for (double y = y0 + halfSizeMP ; y <= y1 - halfSizeMP ; y += size) {
		for (double x = x0 + halfSizeMP ; x <= x1 - halfSizeMP ; x += size) {
			P.pos.set(x, y);
			P.nb = counter;
			counter++;
			box->MP.push_back(P);
		}
	}

	for (size_t p = 0 ; p < box->MP.size() ; p++) {
		box->MP[p].updateCornersFromF();
		//TODO: not using this initialElement
		double invL[2];
		invL[0] = 1.0f / box->Grid.lx;
		invL[1] = 1.0f / box->Grid.ly;

		box->MP[p].initialElement = (int)(trunc(box->MP[p].pos.x * invL[0]) + trunc(box->MP[p].pos.y * invL[1]) * box->Grid.Nx);

	}

	std::cout << "@set_MP_grid::exec, total number of MPs: " << box->MP.size() << std::endl;
	*/
}
