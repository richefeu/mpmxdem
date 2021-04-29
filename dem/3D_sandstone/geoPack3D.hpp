#ifndef GEOPACK3D_HPP_1E13673E
#define GEOPACK3D_HPP_1E13673E

// à voir : journal of the mechanics and physics systems

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <vector>

class GeoPack3D {
public:
  struct Sphere {
    double x, y, z;
    double r;
    Sphere(double x_, double y_, double z_, double r_) {
      x = x_;
      y = y_;
      z = z_;
      r = r_;
    }
  };

  double rmin;
  double rmax;
  double gapTol;
  double distNeighbor; // distance to be said Neighbor (to belong to the neighbor-list)
  double distMin; // Minimum distance between particles
  int max;
  int k;

  std::vector<Sphere> sample;
  std::vector<std::vector<int>> prox;
  std::vector<int> boundaries;
  std::vector<int> active;

  double xmin, xmax;
  double ymin, ymax;
  double zmin, zmax;

  // Ctor
  GeoPack3D() {
    rmin = rmax = 0.0;
    gapTol = distNeighbor = distMin = 0.0;
    max = 0;
    k = 0;
    xmin = xmax = ymin = ymax = zmin = zmax = 0.0;
  }

  GeoPack3D(double rmin_, double rmax_, int k_, double xmin_, double xmax_, double ymin_, double ymax_, double zmin_,
            double zmax_, double gap_ = 0.0, int max_ = 0) {
    parameters(rmin_, rmax_, k_, xmin_, xmax_, ymin_, ymax_, zmin_, zmax_, gap_, max_);
  }

  void parameters(double rmin_, double rmax_, int k_, double xmin_, double xmax_, double ymin_, double ymax_,
                  double zmin_, double zmax_, double gapTol_ = 0.0, int max_ = 0) {
    rmin = rmin_;
    rmax = rmax_;
    gapTol = gapTol_;
    distNeighbor = 2.0 * rmax + gapTol;
    distMin = 0.0;
    k = k_;

    // Domain
    xmin = xmin_;
    xmax = xmax_;
    ymin = ymin_;
    ymax = ymax_;
    zmin = zmin_;
    zmax = zmax_;

    max = max_;
    if (max == 0) {
      int nx = floor((xmax - xmin) / rmin);
      int ny = floor((ymax - ymin) / rmin);
      int nz = floor((zmax - zmin) / rmin);
      max = nx * ny * nz;
    }
  }

  void seedTime() { srand(time(NULL)); }

  void reActivate(int from = 0, int to = 0) {
    if (to == 0)
      to = sample.size();
    for (int i = from; i < to; i++) {
      active.push_back(i);
    }
  }

  // execute the algorithm
  void exec(bool skipFirstStep = false) {
    // step 1
    if (skipFirstStep == false) {
      double firstr = ran(rmin, rmax);
      Sphere P(ran(xmin + firstr, xmax - firstr), ran(ymin + firstr, ymax - firstr), ran(zmin + firstr, zmax - firstr),
               firstr);
      sample.push_back(P);
      prox.push_back(std::vector<int>());
      active.push_back(sample.size() - 1);
    }

    // step 2
    int count = 0;
    int countMax = (int)floor((xmax - xmin) / (0.5 * (rmin + rmax)));
    while (active.size() > 0 && sample.size() < (size_t)max) {
      int randIndex = rand() % active.size();
      int currentSphere = active[randIndex];
      double packedx = sample[currentSphere].x;
      double packedy = sample[currentSphere].y;
      double packedz = sample[currentSphere].z;
      double packedr = sample[currentSphere].r;

      bool found = false;

      for (int n = 0; n < k; n++) {
        double testr = ran(rmin, rmax);
        double angle1 = ran(0, 2.0 * M_PI);
        double angle2 = ran(-0.5 * M_PI, 0.5 * M_PI);
        double m = ran(testr + packedr + distMin, testr + packedr + distMin + gapTol);
        double ux = cos(angle1);
        double uy = sin(angle1);
        double uz = sin(angle2);
        //double sc = sqrt(ux * ux + uy * uy + uz * uz);
        double testx = packedx + m * ux;
        double testy = packedy + m * uy;
        double testz = packedz + m * uz;

        bool ok = true;
        // boundaries
        if (testx < xmin + testr || testx > xmax - testr || testy < ymin + testr || testy > ymax - testr ||
            testz < zmin + testr || testz > zmax - testr) {
          ok = false;
        }

        // inter-particles
        if (ok == true) {
          for (size_t i = 0; i < prox[currentSphere].size(); i++) {
            int neighborSphere = prox[currentSphere][i];

            double dx = sample[neighborSphere].x - testx;
            double dy = sample[neighborSphere].y - testy;
            double dz = sample[neighborSphere].z - testz;
            double d = sqrt(dx * dx + dy * dy + dz * dz);
            if (d < testr + sample[neighborSphere].r + distMin) {
              ok = false;
              break;
            }
          }
        }

        if (ok == true) {
          found = true;
          Sphere P(testx, testy, testz, testr);
          sample.push_back(P);
          prox.push_back(std::vector<int>());
          int particleIndex = sample.size() - 1;

          for (size_t i = 0; i < sample.size(); i++) {
            if ((int)i == particleIndex)
              continue;
            double dx = fabs(sample[i].x - testx);
            double dy = fabs(sample[i].y - testy);
            double dz = fabs(sample[i].z - testz);
            double d = sqrt(dx * dx + dy * dy + dz * dz);
            if (d < sample[i].r + testr + distNeighbor + distMin) {
              prox[i].push_back(particleIndex);
              prox[particleIndex].push_back(i);
            }
          }

          active.push_back(particleIndex);
          break;
        }
      } // n-loop

      if (!found) {
        for (int i = randIndex; i < (int)active.size() - 1; i++) {
          active[i] = active[i + 1];
        }
        active.pop_back();
      }

      count++;
      if (count >= countMax) {
        count = 0;
        std::cout << "Number of spheres packed: " << sample.size() << std::endl;
        std::cout << "Number of active spheres: " << active.size() << std::endl << std::endl;
      }
    } // end-while
  }   // end-method-exec

  void execPeriodic(bool skipFirstStep = false) {
    // step 1
    if (skipFirstStep == false) {
      double firstr = ran(rmin, rmax);
      Sphere P(ran(xmin + firstr, xmax - firstr), ran(ymin + firstr, ymax - firstr), ran(zmin + firstr, zmax - firstr),
               firstr);
      sample.push_back(P);
      prox.push_back(std::vector<int>());
      active.push_back(sample.size() - 1);
    }

    // step 2
    int count = 0;
    int countMax = (int)floor((xmax - xmin) / (0.5 * (rmin + rmax)));
    while (active.size() > 0 && sample.size() < (size_t)max) {
      int randIndex = rand() % active.size();
      int currentSphere = active[randIndex];
      double packedx = sample[currentSphere].x;
      double packedy = sample[currentSphere].y;
      double packedz = sample[currentSphere].z;
      double packedr = sample[currentSphere].r;

      bool found = false;

      for (int n = 0; n < k; n++) {
        double testr = ran(rmin, rmax);
        double angle1 = ran(0, 2.0 * M_PI);
        double angle2 = ran(-0.5 * M_PI, 0.5 * M_PI);
        double m = ran(testr + packedr + distMin, testr + packedr + distMin + gapTol);
        double ux = cos(angle1);
        double uy = sin(angle1);
        double uz = sin(angle2);
        //double sc = sqrt(ux * ux + uy * uy + uz * uz);
        double testx = packedx + m * ux;
        double testy = packedy + m * uy;
        double testz = packedz + m * uz;

        bool ok = true;
        // boundaries
        if (testx < xmin || testx > xmax || testy < ymin || testy > ymax || testz < zmin || testz > zmax) {
          ok = false;
        }
        double dv = 2.0 * rmax + distMin + gapTol; // a verifier
        if (testx < xmin + dv || testx > xmax - dv || testy < ymin + dv || testy > ymax - dv || testz < zmin + dv ||
            testz > zmax - dv) {
          double lx = xmax - xmin;
          double half_lx = 0.5 * lx;
          double ly = ymax - ymin;
          double half_ly = 0.5 * ly;
          double lz = zmax - zmin;
          double half_lz = 0.5 * lz;

          for (size_t i = 0; i < boundaries.size(); i++) {

            int neighborDisk = boundaries[i];

            double dx = sample[neighborDisk].x - testx;
            double dy = sample[neighborDisk].y - testy;
            double dz = sample[neighborDisk].z - testz;

            if (dx > half_lx) {
              dx -= lx;
            } else if (dx < -half_lx) {
              dx += lx;
            }
            if (dy > half_ly) {
              dy -= ly;
            } else if (dy < -half_ly) {
              dy += ly;
            }
            if (dz > half_lz) {
              dz -= lz;
            } else if (dz < -half_lz) {
              dz += lz;
            }

            double d = sqrt(dx * dx + dy * dy + dz * dz);
            if (d < testr + sample[neighborDisk].r + distMin) {
              ok = false;
              break;
            }
          }
        }

        // inter-particles
        if (ok == true) {
          for (size_t i = 0; i < prox[currentSphere].size(); i++) {
            int neighborSphere = prox[currentSphere][i];

            double dx = sample[neighborSphere].x - testx;
            double dy = sample[neighborSphere].y - testy;
            double dz = sample[neighborSphere].z - testz;
            double d = sqrt(dx * dx + dy * dy + dz * dz);
            if (d < testr + sample[neighborSphere].r + distMin) {
              ok = false;
              break;
            }
          }
        }

        if (ok == true) {
          found = true;
          Sphere P(testx, testy, testz, testr);
          sample.push_back(P);
          prox.push_back(std::vector<int>());

          int particleIndex = sample.size() - 1;
          double dv = 2.0 * rmax + distMin + gapTol; // a verifier
          if (testx < xmin + dv || testx > xmax - dv || testy < ymin + dv || testy > ymax - dv || testz < zmin + dv ||
              testz > zmax - dv) {
            boundaries.push_back(particleIndex);
          }

          for (size_t i = 0; i < sample.size(); i++) {
            if ((int)i == particleIndex)
              continue;
            double dx = fabs(sample[i].x - testx);
            double dy = fabs(sample[i].y - testy);
            double dz = fabs(sample[i].z - testz);
            double d = sqrt(dx * dx + dy * dy + dz * dz);
            if (d < sample[i].r + testr + distNeighbor + distMin) {
              prox[i].push_back(particleIndex);
              prox[particleIndex].push_back(i);
            }
          }

          active.push_back(particleIndex);
          break;
        }
      } // n-loop

      if (!found) {
        for (int i = randIndex; i < (int)active.size() - 1; i++) {
          active[i] = active[i + 1];
        }
        active.pop_back();
      }

      count++;
      if (count >= countMax) {
        count = 0;
        std::cout << "Number of spheres packed: " << sample.size() << std::endl;
        std::cout << "Number of active spheres: " << active.size() << std::endl << std::endl;
      }
    } // end-while
  }   // end-execPeriodic

  // Pour visualiser :
  // compiler 'seeSpheres' puis > seeSpheres points.txt
  void save(const char *name) {
    std::ofstream file(name);
    for (size_t i = 0; i < sample.size(); i++) {
      file << sample[i].x << ' ' << sample[i].y << ' ' << sample[i].z << ' ' << sample[i].r << '\n';
    }
  }

  void saveVTK(const char *name) {
    std::ofstream fog(name, std::ios::out);
    if (fog) {
      fog.precision(5);
      fog << std::scientific;
      fog << "# vtk DataFile Version 3.0\n";
      fog << "My grains\n";
      fog << "ASCII\n";
      fog << "DATASET UNSTRUCTURED_GRID\n";
      fog << "POINTS " << sample.size() << " float\n";
      for (size_t i = 0; i < sample.size(); i++)
        fog << sample[i].x << " " << sample[i].y << " " << sample[i].z << '\n';
      fog << "POINT_DATA " << sample.size() << '\n';
      fog << "SCALARS Diameter float\n";
      fog << "LOOKUP_TABLE default\n";
      for (size_t i = 0; i < sample.size(); i++)
        fog << 2 * sample[i].r << '\n';
    }
  }

protected:
  double ran(double min_, double max_) { return min_ + (rand() / (double)RAND_MAX) * (max_ - min_); }
};

#endif /* end of include guard: GEOPACK2D_HPP_1E13673E */
