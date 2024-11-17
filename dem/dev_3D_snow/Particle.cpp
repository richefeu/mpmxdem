#include "AABB.hpp"
#include "kwParser.hpp"
#include "polyhTool.hpp"

#include "Particle.hpp"

Particle::Particle()
    : pos(), vel(), acc(), Q(), vrot(), arot(), I_m(), mass(0.0), volume(0.0), force(), moment(), origPos() {}

void Particle::readShape(std::istream& is, double density) {
  kwParser parser;
  parser.breakStr = ">";
  parser.kwMap["volume"] = __DO__(is) {
    is >> volume;
    mass = volume * density;
  };
  parser.kwMap["I/m"] = __GET__(is, I_m);
  parser.kwMap["obb.extent"] = __GET__(is, obb.extent);
  parser.kwMap["obb.center"] = __GET__(is, obb.center);
  parser.kwMap["obb.e1"] = __GET__(is, obb.e[0]);
  parser.kwMap["obb.e2"] = __GET__(is, obb.e[1]);
  parser.kwMap["obb.e3"] = __GET__(is, obb.e[2]);
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

void Particle::writeShape(std::ostream& os) {
  os << "<\n";
  os << "volume " << volume << '\n';
  os << "I/m " << I_m << '\n';

  os << "obb.extent " << obb.extent << '\n';
  os << "obb.center " << obb.center << '\n';
  os << "obb.e1 " << obb.e[0] << '\n';
  os << "obb.e2 " << obb.e[1] << '\n';
  os << "obb.e3 " << obb.e[2] << '\n';
  os << "nv " << subSpheres.size() << '\n';
  for (size_t v = 0; v < subSpheres.size(); ++v) {
    os << subSpheres[v].localPos << ' ' << subSpheres[v].radius << '\n';
  }
  os << ">\n";
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

  // loop over the points to find the mean point location
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
  u.normalize();
  f.normalize();

  // now build the bounding box extents in the rotated frame
  vec3r minim(1e20, 1e20, 1e20), maxim(-1e20, -1e20, -1e20);
  for (size_t i = 0; i < subSpheres.size(); i++) {
    // size_t i = vertexID[id];
    double ri = subSpheres[i].radius;
    vec3r p_prime(r * subSpheres[i].localPos, u * subSpheres[i].localPos, f * subSpheres[i].localPos);
    if (minim.x > p_prime.x - ri) minim.x = p_prime.x - ri;
    if (minim.y > p_prime.y - ri) minim.y = p_prime.y - ri;
    if (minim.z > p_prime.z - ri) minim.z = p_prime.z - ri;
    if (maxim.x < p_prime.x + ri) maxim.x = p_prime.x + ri;
    if (maxim.y < p_prime.y + ri) maxim.y = p_prime.y + ri;
    if (maxim.z < p_prime.z + ri) maxim.z = p_prime.z + ri;
  }

  // set the center of the OBB to be the average of the
  // minimum and maximum, and the extents be half of the
  // difference between the minimum and maximum
  obb.center = eigvec * (0.5 * (maxim + minim));
  obb.e[0] = r;
  obb.e[1] = u;
  obb.e[2] = f;
  obb.extent = 0.5 * (maxim - minim);
}

// It says wether a point in inside a clump of spheres
bool Particle::inside(const vec3r& point) {
  for (size_t is = 0; is < subSpheres.size(); is++) {
    vec3r b = subSpheres[is].localPos - point;
    double r2 = subSpheres[is].radius * subSpheres[is].radius;
    double l2 = norm2(b);
    if (l2 < r2) return true;
  }
  return false;
}

void Particle::massProperties() {
  std::cout << std::endl;
  std::cout << "Computation of mass properties (volume, mass-center, inertia and body-frame)\n";

  // 1- Get the bounding volume
  double rmax = subSpheres[0].radius;
  AABB box(subSpheres[0].localPos);
  for (size_t is = 1; is < subSpheres.size(); is++) {
    box.add(subSpheres[is].localPos);
    if (subSpheres[is].radius > rmax) rmax = subSpheres[is].radius;
  }
  box.enlarge(rmax);
  double Vbox = (box.max.x - box.min.x) * (box.max.y - box.min.y) * (box.max.z - box.min.z);

  // 2- Monte Carlo integration to compute the volume and the mass-center
  double int_vol = 0.0, vol_err = 0.0;
  vec3r OG;
  std::vector<double> vv(3);
  polyhTool::sobolSequence(-3, vv);  // Initialize the Sobol sequence
  vec3r pt3;
  size_t MCnstep = 10000 * subSpheres.size();
  for (size_t i = 0; i < MCnstep; ++i) {
    polyhTool::sobolSequence(3, vv);
    pt3.set(box.min.x + vv[0] * (box.max.x - box.min.x), box.min.y + vv[1] * (box.max.y - box.min.y),
            box.min.z + vv[2] * (box.max.z - box.min.z));
    if (inside(pt3)) {
      OG += pt3;
      int_vol += 1.0;
      vol_err += 1.0;
    }
  }

  if (int_vol == 0.0) {
    std::cout << "@Particle::massProperties, No point inside!!\n";  // should be FATAL (?)
    return;
  }

  double inv_nstep = 1.0 / (double)MCnstep;
  volume = Vbox * int_vol * inv_nstep;

  vol_err = Vbox * sqrt((vol_err * inv_nstep - (int_vol * inv_nstep) * (int_vol * inv_nstep)) * inv_nstep);

  OG = inv_nstep * Vbox * OG;
  if (volume > 1.0e-20) OG = (1.0 / volume) * OG;

  // 3- Monte Carlo integration to compute (1/m) I(G)
  double I11_m = 0.0;
  double I12_m = 0.0;
  double I13_m = 0.0;
  double I22_m = 0.0;
  double I23_m = 0.0;
  double I33_m = 0.0;

  double lx, ly, lz;
  for (size_t i = 0; i < MCnstep; ++i) {
    polyhTool::sobolSequence(3, vv);
    pt3.set(box.min.x + vv[0] * (box.max.x - box.min.x), box.min.y + vv[1] * (box.max.y - box.min.y),
            box.min.z + vv[2] * (box.max.z - box.min.z));
    if (inside(pt3)) {
      lx = pt3.x - OG.x;
      ly = pt3.y - OG.y;
      lz = pt3.z - OG.z;

      I11_m += (ly * ly + lz * lz);
      I12_m -= (lx * ly);
      I13_m -= (lx * lz);
      I22_m += (lx * lx + lz * lz);
      I23_m -= (ly * lz);
      I33_m += (lx * lx + ly * ly);
    }
  }

  double fact = (Vbox / volume) * inv_nstep;
  I11_m *= fact;
  I12_m *= fact;
  I13_m *= fact;
  I22_m *= fact;
  I23_m *= fact;
  I33_m *= fact;

  mat9r matI_m(I11_m, I12_m, I13_m, I12_m, I22_m, I23_m, I13_m, I23_m, I33_m);

  mat9r VP;  // Eigen vectors
  vec3r D;   // Eigen values
  matI_m.sym_eigen(VP, D);

  // 4- set the precomputed properties
  quat Qtmp;
  Qtmp.set_rot_matrix(VP.c_mtx());
  Qtmp.normalize();

  origPos = OG;
  Q = Qtmp;

  quat Qinv = Qtmp.get_conjugated();
  for (size_t i = 0; i < subSpheres.size(); ++i) {
    subSpheres[i].localPos = Qinv * (subSpheres[i].localPos - OG);
  }
  // position = OG;
  I_m = D;
  fitObb();

  // 5- Display some information
  std::cout << "Number of steps in the Monte Carlo integration: " << MCnstep << std::endl;
  std::cout << "      Estimated error for the volume (err/vol): " << vol_err / volume << std::endl;
  std::cout << "                                        Volume: " << volume << std::endl;
  std::cout << "                                  inertia/mass: " << I_m << std::endl;
  std::cout << "                 Angular position (quaternion): " << Q << std::endl;
  std::cout << "                             Original position: " << origPos << std::endl;
  std::cout << std::endl;
}