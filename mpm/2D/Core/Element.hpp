#ifndef ELEMENT_HPP
#define ELEMENT_HPP

struct element {
  static int nbNodes;

  //    13 12 11 10
  // 	  14 3  2  9
  //    15 0  1  8
  //    4  5  6  7
  // for regularquadlinear we use the inner square only
  // for BsplineCubic we use the inner and outer square
  int I[16];

  element();  // Ctor
};

#endif /* end of include guard: ELEMENT_HPP */
