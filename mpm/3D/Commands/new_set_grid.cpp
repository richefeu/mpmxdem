#include "new_set_grid.hpp"
#include <factory.hpp>
#include "../Core/MPMbox.hpp"
static Registrar<Command, new_set_grid> registrar("new_set_grid");

void new_set_grid::read(std::istream & is)
{
	is >> lengthX >> lengthY >> lengthZ >> spacing;
}

void new_set_grid::exec()
{

	if (box->shapeFunction == nullptr) {
		std::cerr << "@new_set_grid::exec, ShapeFunction has to be set BEFORE set_grid." << std::endl;
		exit(0);
	}

	box->Grid.Nx = floor(lengthX/spacing);
	box->Grid.Ny = floor(lengthY/spacing);
	box->Grid.Nz = floor(lengthZ/spacing);

	//Used when calculating the shape Functions
	box->Grid.lx = spacing;
	box->Grid.ly = spacing;
	box->Grid.lz = spacing;

	// Create the nodes and set their positions
	if (!box->nodes.empty()) box->nodes.clear();
	node N;
	N.mass = 0.0;
	N.q.reset();
	N.f.reset();
	N.fb.reset();
	N.xfixed = false;
	N.yfixed = false;
	N.zfixed = false;
	int counter = 0;
	for(int k = 0; k <= box->Grid.Nz; k++) {
		for (int j = 0 ; j <= box->Grid.Ny ; j++) {
			for (int i = 0 ; i <= box->Grid.Nx ; i++) {
                N.number = counter;
				N.pos.x = i * spacing;
				N.pos.y = j * spacing;
				N.pos.z = k * spacing;
                counter++;
				box->nodes.push_back(N);
				// std::cout<<N.pos<<std::endl;
			}
		}
	}

	// QUA4 elements
	// 2 _ 3
	// |   |
	// 0 _ 1
	if (!box->Elem.empty()) box->Elem.clear();
	element E;
	for (int k = 0;k < box->Grid.Nz; k++) {
		for (int j = 0 ; j < box->Grid.Ny ; j++) {
			for (int i = 0 ; i < box->Grid.Nx ; i++) {
				E.I[0] = (box->Grid.Nx + 1)*(box->Grid.Ny + 1)*k + (box->Grid.Nx + 1) * j + i;
				E.I[1] = (box->Grid.Nx + 1)*(box->Grid.Ny + 1)*k + (box->Grid.Nx + 1) * j + i + 1;
				E.I[2] = (box->Grid.Nx + 1)*(box->Grid.Ny + 1)*k + (box->Grid.Nx + 1) * (j + 1) + i;
				E.I[3] = (box->Grid.Nx + 1)*(box->Grid.Ny + 1)*k + (box->Grid.Nx + 1) * (j + 1) + i + 1;
				E.I[4] = (box->Grid.Nx + 1)*(box->Grid.Ny + 1)*(k + 1) + (box->Grid.Nx + 1) * j + i;
				E.I[5] = (box->Grid.Nx + 1)*(box->Grid.Ny + 1)*(k + 1) + (box->Grid.Nx + 1) * j + i + 1;
				E.I[6] = (box->Grid.Nx + 1)*(box->Grid.Ny + 1)*(k + 1) + (box->Grid.Nx + 1) * (j + 1) + i;
				E.I[7] = (box->Grid.Nx + 1)*(box->Grid.Ny + 1)*(k + 1) + (box->Grid.Nx + 1) * (j + 1) + i + 1;
				box->Elem.push_back(E);
			}
		}
	}
	//TODO. Set number correct nodes in element for the appropriate Shape function

    for (size_t i = 0; i < box->nodes.size(); i++) {
        box->liveNodeNum.push_back(i);
    }
}
