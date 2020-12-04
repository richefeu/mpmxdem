#ifndef NEIGHBOR_HPP_55726DF1
#define NEIGHBOR_HPP_55726DF1

struct Neighbor
{
	int PointNumber;
	//double dist0;
	//bool touch;
	
	// forces on each corner
	double fn[4], dn[4];
	double ft[4], dt[4];
	
	Neighbor(); // Ctor
};

#endif /* end of include guard: NEIGHBOR_HPP_55726DF1 */
