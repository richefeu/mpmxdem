#pragma once

//
// Class and functions related to the FE mesh
//

#include <cstddef>

struct element {
  static size_t nbNodes; // it will be 4 or 16

  //    13 12 11 10
  // 	  14 3  2  9
  //    15 0  1  8
  //    4  5  6  7
  // for RegularQuadLinear, we use the inner square only
  // for BSpline, we use the inner and outer square

  // Position of each of the 16 nodes relative to the node 0 of the element,
  // counted in cells, following the numbering drawn above. The first four
  // entries are exactly the QUA4 layout, so the same table serves both kinds of
  // element -- element::nbNodes says how many to read.
  //
  // This is the ONE place where that numbering is written down. MPMbox::buildGrid
  // fills element::I with it, and BSpline reads its shape functions in the same
  // order: the two must agree, so they must not be two tables.
  static const int dxOff[16];
  static const int dyOff[16];

  // clang-format off
  size_t I[16]{
    0, 0, 0, 0, 
    0, 0, 0, 0, 
    0, 0, 0, 0, 
    0, 0, 0, 0
  };
  // clang-format on

  element(); // Ctor
};
