#ifndef NEIGHBOR_HPP
#define NEIGHBOR_HPP

struct Neighbor {
  int PointNumber;  // Material Point number

  double fn, dn;
  double ft, dt;

  Neighbor();  // Ctor
};

#endif /* end of include guard: NEIGHBOR_HPP */
