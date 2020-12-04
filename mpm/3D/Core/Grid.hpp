#ifndef GRID_HPP_D88842EE
#define GRID_HPP_D88842EE

struct grid
{
	double lx,ly,lz;     // Size of QUA4 elements (in case of regular grid)
	int Nx,Ny,Nz;        // Number of grid-elements (QUA4) along x and y directions

    grid();              // Ctor
};

#endif /* end of include guard: GRID_HPP_D88842EE */
