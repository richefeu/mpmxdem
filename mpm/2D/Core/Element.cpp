#include "Element.hpp"

size_t element::nbNodes = 0;

element::element() {}

// See the drawing in Element.hpp. Node 0 of the element is the origin.
const int element::dxOff[16] = {0, 1, 1, 0, -1, 0, 1, 2, 2, 2, 2, 1, 0, -1, -1, -1};
const int element::dyOff[16] = {0, 0, 1, 1, -1, -1, -1, -1, 0, 1, 2, 2, 2, 2, 1, 0};
