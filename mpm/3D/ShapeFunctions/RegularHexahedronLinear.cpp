#include "RegularHexahedronLinear.hpp"

#include <../../common/factory.hpp>
static Registrar<ShapeFunction, RegularHexahedronLinear> registrar("RegularHexahedronLinear");

void RegularHexahedronLinear::computeInterpolationValues (MPMbox & MPM, size_t p)
{
	MPM.MP[p].e = (int)(trunc(MPM.MP[p].pos.x / MPM.Grid.lx) + trunc(MPM.MP[p].pos.y / MPM.Grid.ly) * MPM.Grid.Nx + trunc(MPM.MP[p].pos.z / MPM.Grid.lz) * MPM.Grid.Nx * MPM.Grid.Ny); // Equation 22
	// int delvar;
	// std::cout<<MPM.MP[p].e<<std::endl;

	int *I = &(MPM.Elem[ MPM.MP[p].e ].I[0]);

	double invx = 1.0 / MPM.Grid.lx;
	double invy = 1.0 / MPM.Grid.ly;
	double invz = 1.0 / MPM.Grid.lz;

	double x0 = MPM.MP[p].pos.x - MPM.nodes[ I[0] ].pos.x;
	//std::cout<<"cp2.3"<<std::endl;
	double y0 = MPM.MP[p].pos.y - MPM.nodes[ I[0] ].pos.y;
	double z0 = MPM.MP[p].pos.z - MPM.nodes[ I[0] ].pos.z;
	double x1 = MPM.MP[p].pos.x - MPM.nodes[ I[1] ].pos.x;
	double y1 = MPM.MP[p].pos.y - MPM.nodes[ I[1] ].pos.y;
	double z1 = MPM.MP[p].pos.z - MPM.nodes[ I[1] ].pos.z;
	double x2 = MPM.MP[p].pos.x - MPM.nodes[ I[2] ].pos.x;
	double y2 = MPM.MP[p].pos.y - MPM.nodes[ I[2] ].pos.y;
	double z2 = MPM.MP[p].pos.z - MPM.nodes[ I[2] ].pos.z;
	double x3 = MPM.MP[p].pos.x - MPM.nodes[ I[3] ].pos.x;
	double y3 = MPM.MP[p].pos.y - MPM.nodes[ I[3] ].pos.y;
	double z3 = MPM.MP[p].pos.z - MPM.nodes[ I[3] ].pos.z;
	double x4 = MPM.MP[p].pos.x - MPM.nodes[ I[4] ].pos.x;
	double y4 = MPM.MP[p].pos.y - MPM.nodes[ I[4] ].pos.y;
	double z4 = MPM.MP[p].pos.z - MPM.nodes[ I[4] ].pos.z;
	double x5 = MPM.MP[p].pos.x - MPM.nodes[ I[5] ].pos.x;
	double y5 = MPM.MP[p].pos.y - MPM.nodes[ I[5] ].pos.y;
	double z5 = MPM.MP[p].pos.z - MPM.nodes[ I[5] ].pos.z;
	double x6 = MPM.MP[p].pos.x - MPM.nodes[ I[6] ].pos.x;
	double y6 = MPM.MP[p].pos.y - MPM.nodes[ I[6] ].pos.y;
	double z6 = MPM.MP[p].pos.z - MPM.nodes[ I[6] ].pos.z;
	double x7 = MPM.MP[p].pos.x - MPM.nodes[ I[7] ].pos.x;
	double y7 = MPM.MP[p].pos.y - MPM.nodes[ I[7] ].pos.y;
	double z7 = MPM.MP[p].pos.z - MPM.nodes[ I[7] ].pos.z;
	//std::cout<<"cp2.4"<<std::endl;
	double phix0 = 1.0 - fabs(x0) * invx;
	double phiy0 = 1.0 - fabs(y0) * invy;
	double phiz0 = 1.0 - fabs(z0) * invz;
	double phix1 = 1.0 - fabs(x1) * invx;
	double phiy1 = 1.0 - fabs(y1) * invy;
	double phiz1 = 1.0 - fabs(z1) * invz;
	double phix2 = 1.0 - fabs(x2) * invx;
	double phiy2 = 1.0 - fabs(y2) * invy;
	double phiz2 = 1.0 - fabs(z2) * invz;
	double phix3 = 1.0 - fabs(x3) * invx;
	double phiy3 = 1.0 - fabs(y3) * invy;
	double phiz3 = 1.0 - fabs(z3) * invz;
	double phix4 = 1.0 - fabs(x4) * invx;
	double phiy4 = 1.0 - fabs(y4) * invy;
	double phiz4 = 1.0 - fabs(z4) * invz;
	double phix5 = 1.0 - fabs(x5) * invx;
	double phiy5 = 1.0 - fabs(y5) * invy;
	double phiz5 = 1.0 - fabs(z5) * invz;
	double phix6 = 1.0 - fabs(x6) * invx;
	double phiy6 = 1.0 - fabs(y6) * invy;
	double phiz6 = 1.0 - fabs(z6) * invz;
	double phix7 = 1.0 - fabs(x7) * invx;
	double phiy7 = 1.0 - fabs(y7) * invy;
	double phiz7 = 1.0 - fabs(z7) * invz;

	MPM.MP[p].N[0] = phix0 * phiy0 * phiz0;
	MPM.MP[p].N[1] = phix1 * phiy1 * phiz1;
	MPM.MP[p].N[2] = phix2 * phiy2 * phiz2;
	MPM.MP[p].N[3] = phix3 * phiy3 * phiz3;
	MPM.MP[p].N[4] = phix4 * phiy4 * phiz4;
	MPM.MP[p].N[5] = phix5 * phiy5 * phiz5;
	MPM.MP[p].N[6] = phix6 * phiy6 * phiz6;
	MPM.MP[p].N[7] = phix7 * phiy7 * phiz7;

	double tol = 0.01;
	double sum =0;
	for (int i = 0; i<8;i++) {
		sum += MPM.MP[p].N[i];
	}
    //double sum = MPM.MP[p].N[0] + MPM.MP[p].N[1] + MPM.MP[p].N[2] + MPM.MP[p].N[3];

    if (fabs(sum - 1.0) > tol) // (sum >1+tol ||sum <1-tol)
	{
		std::cout << "SUM OF SHAPE FUNCTIONS IS NOT 1, MPs COULD BE OUTSIDE THE GRID.\nCHECK MP NUMBER: " << p << "\tPOSITION: " << MPM.MP[p].pos << std::endl;
		exit(EXIT_FAILURE);
	}


	MPM.MP[p].gradN[0].x = -copysign(1.0, x0) * invx * phiy0 * phiz0;
	MPM.MP[p].gradN[1].x = -copysign(1.0, x1) * invx * phiy1 * phiz1;
	MPM.MP[p].gradN[2].x = -copysign(1.0, x2) * invx * phiy2 * phiz2;
	MPM.MP[p].gradN[3].x = -copysign(1.0, x3) * invx * phiy3 * phiz3;
	MPM.MP[p].gradN[4].x = -copysign(1.0, x4) * invx * phiy4 * phiz4;
	MPM.MP[p].gradN[5].x = -copysign(1.0, x5) * invx * phiy5 * phiz5;
	MPM.MP[p].gradN[6].x = -copysign(1.0, x6) * invx * phiy6 * phiz6;
	MPM.MP[p].gradN[7].x = -copysign(1.0, x7) * invx * phiy7 * phiz7;

	MPM.MP[p].gradN[0].y = -copysign(1.0, y0) * invy * phix0 * phiz0;
	MPM.MP[p].gradN[1].y = -copysign(1.0, y1) * invy * phix1 * phiz1;
	MPM.MP[p].gradN[2].y = -copysign(1.0, y2) * invy * phix2 * phiz2;
	MPM.MP[p].gradN[3].y = -copysign(1.0, y3) * invy * phix3 * phiz3;
	MPM.MP[p].gradN[4].y = -copysign(1.0, y4) * invy * phix4 * phiz4;
	MPM.MP[p].gradN[5].y = -copysign(1.0, y5) * invy * phix5 * phiz5;
	MPM.MP[p].gradN[6].y = -copysign(1.0, y6) * invy * phix6 * phiz6;
	MPM.MP[p].gradN[7].y = -copysign(1.0, y7) * invy * phix7 * phiz7;

	MPM.MP[p].gradN[0].z = -copysign(1.0, z0) * invz * phix0 * phiy0;
	MPM.MP[p].gradN[1].z = -copysign(1.0, z1) * invz * phix1 * phiy1;
	MPM.MP[p].gradN[2].z = -copysign(1.0, z2) * invz * phix2 * phiy2;
	MPM.MP[p].gradN[3].z = -copysign(1.0, z3) * invz * phix3 * phiy3;
	MPM.MP[p].gradN[4].z = -copysign(1.0, z4) * invz * phix4 * phiy4;
	MPM.MP[p].gradN[5].z = -copysign(1.0, z5) * invz * phix5 * phiy5;
	MPM.MP[p].gradN[6].z = -copysign(1.0, z6) * invz * phix6 * phiy6;
	MPM.MP[p].gradN[7].z = -copysign(1.0, z7) * invz * phix7 * phiy7;

}
