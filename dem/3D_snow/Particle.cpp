#include "kwParser.hpp"

#include "Particle.hpp"

Particle::Particle() : pos(), vel(), acc(), Q(), vrot(), arot(), inertia(), mass(0.0), force(), moment() {}

void Particle::readShape(std::istream& is) {
  kwParser parser;
  parser.breakStr = ">";
  // parser.kwMap["radius"] = __GET__(is, radius);
  parser.kwMap["volume"] = __GET__(is, volume);
  parser.kwMap["I/m"] = __GET__(is, I_m);
  parser.kwMap["obb.extent"] = __GET__(is, obb.extent);
  parser.kwMap["obb.center"] = __GET__(is, obb.center);
  parser.kwMap["obb.e1"] = __GET__(is, obb.e[0]);
  parser.kwMap["obb.e2"] = __GET__(is, obb.e[1]);
  parser.kwMap["obb.e3"] = __GET__(is, obb.e[2]);
  // parser.kwMap["OBBtreeLevel"] = __GET__(is, OBBtreeLevel);
  parser.kwMap["position"] = __GET__(is, pos);
  parser.kwMap["orientation"] = __GET__(is, Q);
  // parser.kwMap["MCnstep"] = __GET__(is, MCnstep);
  parser.kwMap["nv"] = __DO__(is) {
    size_t nv;
    is >> nv;
    Sphere S;
    for (size_t v = 0; v < nv; ++v) {
      is >> S.localPos >> S.radius;
      subSpheres.push_back(S);
    }
  };

  parser.parse(is);

  // Limit MCnstep value within a reasonnable range
  // if (MCnstep > 10000000) MCnstep = 10000000;
  // if (MCnstep < 1000) MCnstep = 1000;
}

// See pdf document "Minimum-Area Rectangle Containing a Convex Polygon" (@see
// http://www.geometrictools.com/)
void Particle::fitObb() {
  if (subSpheres.empty()) {
    std::cerr << "@Particle::fitObb, No spheres in the clump\n";
    return;
  }

  // ==== Build the covariance matrix
  vec3r mu;
  mat9r C;

  // loop over the points to find the mean point
  // location
  for (size_t i = 0; i < subSpheres.size(); i++) {
    mu += subSpheres[i].localPos;
  }
  mu /= (double)subSpheres.size();

  // loop over the points again to build the
  // covariance matrix.  Note that we only have
  // to build terms for the upper trianglular
  // portion since the matrix is symmetric
  double cxx = 0.0, cxy = 0.0, cxz = 0.0, cyy = 0.0, cyz = 0.0, czz = 0.0;
  for (size_t i = 0; i < subSpheres.size(); i++) {
    // size_t i = vertexID[id];

    vec3r p = subSpheres[i].localPos;
    cxx += p.x * p.x - mu.x * mu.x;
    cxy += p.x * p.y - mu.x * mu.y;
    cxz += p.x * p.z - mu.x * mu.z;
    cyy += p.y * p.y - mu.y * mu.y;
    cyz += p.y * p.z - mu.y * mu.z;
    czz += p.z * p.z - mu.z * mu.z;
  }

  // now build the covariance matrix
  C.xx = cxx;
  C.xy = cxy;
  C.xz = cxz;
  C.yx = cxy;
  C.yy = cyy;
  C.yz = cyz;
  C.zx = cxz;
  C.zy = cyz;
  C.zz = czz;

  // ==== set the OBB parameters from the covariance matrix
  // extract the eigenvalues and eigenvectors from C
  mat9r eigvec;
  vec3r eigval;
  C.sym_eigen(eigvec, eigval);

  // find the right, up and forward vectors from the eigenvectors
  vec3r r(eigvec.xx, eigvec.yx, eigvec.zx);
  vec3r u(eigvec.xy, eigvec.yy, eigvec.zy);
  vec3r f(eigvec.xz, eigvec.yz, eigvec.zz);
  r.normalize();
  u.normalize(), f.normalize();

  // now build the bounding box extents in the rotated frame
  vec3r minim(1e20, 1e20, 1e20), maxim(-1e20, -1e20, -1e20);
  for (size_t i = 0; i < subSpheres.size(); i++) {
    // size_t i = vertexID[id];
    vec3r p_prime(r * subSpheres[i].localPos, u * subSpheres[i].localPos, f * subSpheres[i].localPos);
    if (minim.x > p_prime.x) minim.x = p_prime.x;
    if (minim.y > p_prime.y) minim.y = p_prime.y;
    if (minim.z > p_prime.z) minim.z = p_prime.z;
    if (maxim.x < p_prime.x) maxim.x = p_prime.x;
    if (maxim.y < p_prime.y) maxim.y = p_prime.y;
    if (maxim.z < p_prime.z) maxim.z = p_prime.z;
  }

  // set the center of the OBB to be the average of the
  // minimum and maximum, and the extents be half of the
  // difference between the minimum and maximum
  obb.center = eigvec * (0.5 * (maxim + minim));
  obb.e[0] = r;
  obb.e[1] = u;
  obb.e[2] = f;
  obb.extent = 0.5 * (maxim - minim);

  double rmax = subSpheres[0].radius;
  for (size_t i = 1; i < subSpheres.size(); i++) {
    if (subSpheres[i].radius > rmax) rmax = subSpheres[i].radius;
  }
  obb.enlarge(rmax);  // Add the Minskowski radius
}


void Particle::massProperties() {
	
}