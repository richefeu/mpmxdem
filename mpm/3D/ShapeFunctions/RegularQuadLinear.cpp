#include "RegularQuadLinear.hpp"
#include <../../common/factory.hpp>
static Registrar<ShapeFunction, RegularQuadLinear> registrar("RegularQuadLinear");

void RegularQuadLinear::computeInterpolationValues (MPMbox & MPM, size_t p)
{
	MPM.MP[p].e = (int)(trunc(MPM.MP[p].pos.x/MPM.Grid.lx)+trunc(MPM.MP[p].pos.y/MPM.Grid.ly)*MPM.Grid.Nx); // Equation 22
	int *I = &(MPM.Elem[ MPM.MP[p].e ].I[0]);

	double invx = 1.0 / MPM.Grid.lx;
	double invy = 1.0 / MPM.Grid.ly;

	double x0 = MPM.MP[p].pos.x - MPM.nodes[ I[0] ].pos.x;
	double y0 = MPM.MP[p].pos.y - MPM.nodes[ I[0] ].pos.y;
	double x1 = MPM.MP[p].pos.x - MPM.nodes[ I[1] ].pos.x;
	double y1 = MPM.MP[p].pos.y - MPM.nodes[ I[1] ].pos.y;
	double x2 = MPM.MP[p].pos.x - MPM.nodes[ I[2] ].pos.x;
	double y2 = MPM.MP[p].pos.y - MPM.nodes[ I[2] ].pos.y;
	double x3 = MPM.MP[p].pos.x - MPM.nodes[ I[3] ].pos.x;
	double y3 = MPM.MP[p].pos.y - MPM.nodes[ I[3] ].pos.y;

	double phix0 = 1.0 - fabs(x0) * invx;
	double phiy0 = 1.0 - fabs(y0) * invy;
	double phix1 = 1.0 - fabs(x1) * invx;
	double phiy1 = 1.0 - fabs(y1) * invy;
	double phix2 = 1.0 - fabs(x2) * invx;
	double phiy2 = 1.0 - fabs(y2) * invy;
	double phix3 = 1.0 - fabs(x3) * invx;
	double phiy3 = 1.0 - fabs(y3) * invy;

	MPM.MP[p].N[0] = phix0 * phiy0;
	MPM.MP[p].N[1] = phix1 * phiy1;
	MPM.MP[p].N[2] = phix2 * phiy2;
	MPM.MP[p].N[3] = phix3 * phiy3;

	//double tol = 0.0001;
	//double sum = MPM.MP[p].N[0] + MPM.MP[p].N[1] + MPM.MP[p].N[2] + MPM.MP[p].N[3];

    //if (fabs(sum - 1.0) > tol) // (sum >1+tol ||sum <1-tol)
	//{
	//	std::cout << "SUM OF SHAPE FUNCTIONS IS NOT 1, MPs COULD BE OUTSIDE THE GRID.\nCHECK MP NUMBER: " << p << "\tPOSITION: " << MPM.MP[p].pos << std::endl;
	//	exit(EXIT_FAILURE);
	//}

	MPM.MP[p].gradN[0].x = -copysign(1.0, x0) * invx * phiy0;
	MPM.MP[p].gradN[1].x = -copysign(1.0, x1) * invx * phiy1;
	MPM.MP[p].gradN[2].x = -copysign(1.0, x2) * invx * phiy2;
	MPM.MP[p].gradN[3].x = -copysign(1.0, x3) * invx * phiy3;

	MPM.MP[p].gradN[0].y = -copysign(1.0, y0) * invy * phix0;
	MPM.MP[p].gradN[1].y = -copysign(1.0, y1) * invy * phix1;
	MPM.MP[p].gradN[2].y = -copysign(1.0, y2) * invy * phix2;
	MPM.MP[p].gradN[3].y = -copysign(1.0, y3) * invy * phix3;

}
