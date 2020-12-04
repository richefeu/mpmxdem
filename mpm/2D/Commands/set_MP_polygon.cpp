#include "set_MP_polygon.hpp"

#include <ConstitutiveModels/ConstitutiveModel.hpp>
#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>

#include <factory.hpp>
static Registrar<Command, set_MP_polygon> registrar("set_MP_polygon");

void set_MP_polygon::read(std::istream& is) {
  is >> groupNb >> nbVertices >> modelName >> rho >> size;
  for (int i = 0; i < nbVertices; i++) {
    is >> vertex;
    vertices.push_back(vertex);
  }
}

void set_MP_polygon::exec() {
  /// @see http://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/

  double halfSizeMP = 0.5 * size;

  auto itCM = box->models.find(modelName);
  if (itCM == box->models.end()) {
    std::cerr << "@set_MP_polygon::exec, model " << modelName << " not found" << std::endl;
  }
  ConstitutiveModel* CM = itCM->second;

  MaterialPoint P(groupNb, size, rho, CM);
  // finding max and min in each direction
  double maxx(vertices[0].x), minx(vertices[0].x), maxy(vertices[0].y), miny(vertices[0].y);
  for (size_t i = 0; i < vertices.size(); i++) {
    // for the max
    if (vertices[i].x > maxx) maxx = vertices[i].x;
    if (vertices[i].y > maxy) maxy = vertices[i].y;
    // for the min
    if (vertices[i].x < minx) minx = vertices[i].x;
    if (vertices[i].y < miny) miny = vertices[i].y;
  }

  // using the max and min to fill the geometry
  for (double y = miny + halfSizeMP; y <= maxy - halfSizeMP; y += size) {
    for (double x = minx + halfSizeMP; x <= maxx - halfSizeMP; x += size) {
      vec2r point(x, y);
      if (isInside(vertices, nbVertices, point)) {
        P.pos.set(x, y);
        box->MP.push_back(P);
      }
    }
  }
  std::cout << "@set_MP_polygon::exec, total number of MPs: " << box->MP.size() << std::endl;
}

// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool set_MP_polygon::onSegment(vec2r p, vec2r q, vec2r r) {
  if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) && q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
    return true;
  return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
double set_MP_polygon::orientation(vec2r p, vec2r q, vec2r r) {
  double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

  if (val == 0) return 0;    // colinear
  return (val > 0) ? 1 : 2;  // clock or counterclock wise
}

// The function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool set_MP_polygon::doIntersect(vec2r p1, vec2r q1, vec2r p2, vec2r q2) {
  // Find the four orientations needed for general and
  // special cases
  double o1 = orientation(p1, q1, p2);
  double o2 = orientation(p1, q1, q2);
  double o3 = orientation(p2, q2, p1);
  double o4 = orientation(p2, q2, q1);

  // General case
  if (o1 != o2 && o3 != o4) return true;

  // Special Cases
  // p1, q1 and p2 are colinear and p2 lies on segment p1q1
  if (o1 == 0 && onSegment(p1, p2, q1)) return true;

  // p1, q1 and p2 are colinear and q2 lies on segment p1q1
  if (o2 == 0 && onSegment(p1, q2, q1)) return true;

  // p2, q2 and p1 are colinear and p1 lies on segment p2q2
  if (o3 == 0 && onSegment(p2, p1, q2)) return true;

  // p2, q2 and q1 are colinear and q1 lies on segment p2q2
  if (o4 == 0 && onSegment(p2, q1, q2)) return true;

  return false;  // Doesn't fall in any of the above cases
}

// Returns true if the point p lies inside the polygon[] with n vertices
bool set_MP_polygon::isInside(std::vector<vec2r> polygon, int n, vec2r p) {
  const int inf = 100000;

  // There must be at least 3 vertices in polygon[]
  if (n < 3) return false;

  // Create a point for line segment from p to infinite
  vec2r extreme(inf, p.y);

  // Count intersections of the above line with sides of polygon
  int count = 0, i = 0;
  do {
    int next = (i + 1) % n;

    // Check if the line segment from 'p' to 'extreme' intersects
    // with the line segment from 'polygon[i]' to 'polygon[next]'
    if (doIntersect(polygon[i], polygon[next], p, extreme)) {
      // If the point 'p' is colinear with line segment 'i-next',
      // then check if it lies on segment. If it lies, return true,
      // otherwise false
      if (orientation(polygon[i], p, polygon[next]) == 0) return onSegment(polygon[i], p, polygon[next]);

      count++;
    }
    i = next;
  } while (i != 0);

  // Return true if count is odd, false otherwise
  return count & 1;  // Same as (count%2 == 1)
}
