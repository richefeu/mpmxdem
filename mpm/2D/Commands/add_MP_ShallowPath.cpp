#include "add_MP_ShallowPath.hpp"

#include "ConstitutiveModels/ConstitutiveModel.hpp"
#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

void add_MP_ShallowPath::read(std::istream& is) {
  is >> groupNb >> nbPathPoints >> height >> modelName >> rho >> size;
  for (int i = 0; i < nbPathPoints; i++) {
    is >> pathPoint;
    pathPoints.push_back(pathPoint);
  }
}

void add_MP_ShallowPath::exec() {
  /// @see http://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/

  double halfSizeMP = 0.5 * size;

  auto itCM = box->models.find(modelName);
  if (itCM == box->models.end()) {
    std::cerr << "@add_MP_ShallowPath::exec, model " << modelName << " not found" << std::endl;
  }
  ConstitutiveModel* CM = itCM->second;

  // loop over the path points
  for (size_t i = 0; i < pathPoints.size() - 1; i++) {
    // vector from beginning to end is named vecA
    vec2r vecA = pathPoints[i + 1] - pathPoints[i];

    double maxx = vecA * vec2r::unit_x();

    for (double x = pathPoints[i].x + halfSizeMP; x <= maxx - halfSizeMP; x += size) {
      double ypos = lineEquation(pathPoints[i], pathPoints[i + 1], x);
      for (double y = ypos + halfSizeMP; y <= ypos + height - halfSizeMP; y += size) {
        MaterialPoint P(groupNb, size, rho, CM);
        P.pos.x = x;
        P.pos.y = y;
        box->MP.push_back(P);
      }
    }
  }
}

double add_MP_ShallowPath::lineEquation(const vec2r& point1, const vec2r& point2, const double xpos) {
  double slope = (point2.y - point1.y) / (point2.x - point1.x);
  return slope * (xpos - point1.x) + point1.y;
}
