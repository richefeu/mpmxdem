#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include "mat9.hpp"
#define __FORMATED(P, W, V) std::fixed << std::setprecision(P) << std::setw(W) << std::left << (V)

int main() {
  std::vector<double> time;
  std::vector<mat9r> stress;
  std::vector<mat9r> strain;

  std::ifstream file("stress.out.txt");
  double t;
  mat9r M;
  while (file >> t >> M) {
    time.push_back(t);
    stress.push_back(M);
  }
  std::ifstream file2("strain.out.txt");
  while (file2 >> t >> M) {
    strain.push_back(M);
  }

  // ===============

#if 0
  // force the stress symmetry
  for (size_t i = 0; i < stress.size(); i++) {
    stress[i].xy = 0.5 * (stress[i].xy + stress[i].yx);
    stress[i].yx = stress[i].xy;
    stress[i].xz = 0.5 * (stress[i].xz + stress[i].zx);
    stress[i].zx = stress[i].xz;
    stress[i].yz = 0.5 * (stress[i].yz + stress[i].zy);
    stress[i].zy = stress[i].yz;
  }
#endif

  std::ofstream out("processed.txt");
  for (size_t i = 0; i < stress.size(); i++) {

    double sigOct = (stress[i].xx + stress[i].yy + stress[i].zz) / 3.0;
    double d12 = stress[i].xx - stress[i].yy;
    double d23 = stress[i].yy - stress[i].zz;
    double d13 = stress[i].zz - stress[i].xx;
    double tauOct = sqrt(d12 * d12 + d23 * d23 + d13 * d13) / 3.0;
    double epsOct = (strain[i].xx + strain[i].yy + strain[i].zz) / 3.0;
    d12 = strain[i].xx - strain[i].yy;
    d23 = strain[i].yy - strain[i].zz;
    d13 = strain[i].zz - strain[i].xx;
    double gammaOct = 2.0 * sqrt(d12 * d12 + d23 * d23 + d13 * d13) / 3.0;

    vec3r u(1.0, 1.0, 1.0);
    u.normalize();
    vec3r s(stress[i].xx - sigOct, stress[i].yy - sigOct, stress[i].zz - sigOct);

    double sx = (sqrt(3.0) / 2.0) * fabs(s.y - s.z);
    double sy = s.x - 0.5 * (s.y + s.z);
    double Lode = atan(sx / sy) * 180.0 / M_PI;

    out << __FORMATED(2, 8, time[i]) << ' ' << __FORMATED(6, 12, sx) << ' ' << __FORMATED(6, 12, sy) << ' '
        << __FORMATED(6, 12, epsOct) << ' ' << __FORMATED(6, 12, gammaOct) << ' ' << __FORMATED(6, 12, sigOct) << ' '
        << __FORMATED(6, 12, tauOct) << ' ' << __FORMATED(6, 12, Lode) << '\n';
  }

  return 0;
}
