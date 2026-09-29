#include "Runout.hpp"

#include <iomanip>
#include <limits>

#include "PostSession.hpp"

void Runout::read(std::istream &is) { is >> filename >> xWall >> yFloor >> L0 >> H0; }

void Runout::begin() {
  file.open(session->outPath(filename).c_str());
  file << "# Runout of the column collapse\n";
  file << "# xWall = " << xWall << "  yFloor = " << yFloor << "  L0 = " << L0 << "  H0 = " << H0 << '\n';
  file << "# 1:iconf 2:t 3:xf 4:(xf-xWall)/L0 5:hf 6:(hf-yFloor)/H0 "
          "7:xG 8:yG 9:Ekin 10:vmax 11:nbMP\n";
}

void Runout::exec() {
  const size_t nbMP = session->Conf.MP.size();
  if (nbMP == 0) { return; }

  double xf = -std::numeric_limits<double>::max();
  double hf = -std::numeric_limits<double>::max();

  double mtot = 0.0;
  double xG   = 0.0;
  double yG   = 0.0;
  double Ekin = 0.0;
  double vmax = 0.0;

  for (size_t p = 0; p < nbMP; p++) {
    for (size_t c = 0; c < 4; c++) {
      if (session->Data[p].corner[c].x > xf) { xf = session->Data[p].corner[c].x; }
      if (session->Data[p].corner[c].y > hf) { hf = session->Data[p].corner[c].y; }
    }

    const double m = session->Conf.MP[p].mass;
    mtot += m;
    xG += m * session->Conf.MP[p].pos.x;
    yG += m * session->Conf.MP[p].pos.y;

    const double v2 = session->Conf.MP[p].vel * session->Conf.MP[p].vel;
    Ekin += 0.5 * m * v2;
    if (v2 > vmax) { vmax = v2; }
  }

  if (mtot > 0.0) {
    xG /= mtot;
    yG /= mtot;
  }
  vmax = sqrt(vmax);

  file << session->confNum << ' ' << session->confTime << ' ' << xf << ' ' << (xf - xWall) / L0 << ' ' << hf << ' '
       << (hf - yFloor) / H0 << ' ' << xG << ' ' << yG << ' ' << Ekin << ' ' << vmax << ' ' << nbMP << std::endl;
}

void Runout::end() {
  if (file.is_open()) { file.close(); }
}
