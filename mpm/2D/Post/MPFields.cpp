#include "MPFields.hpp"

#include <fstream>

#include "PostSession.hpp"

void MPFields::read(std::istream &is) { is >> baseName >> xmin >> ymin >> xmax >> ymax; }

void MPFields::exec() {
  std::string name = session->outPath(baseName + std::to_string(session->confNum) + ".txt");
  std::ofstream file(name.c_str());
  if (!file) {
    Logger::warn("@MPFields::exec, cannot write '{}'", name);
    return;
  }

  file << "# conf " << session->confNum << "   t = " << session->confTime << " s\n";
  // Beware: ProcessedDataMP::strain holds the deformation gradient F, not a
  // strain -- see MPMbox::postProcess, 'Data[p].strain = MP[p].F'. The total
  // strain is taken from the Material Point itself.
  file << "# 1:nb 2:group 3:x 4:y 5:vx 6:vy "
          "7:sig_xx 8:sig_xy 9:sig_yx 10:sig_yy 11:sig_zz 12:sig_xz 13:sig_yz "
          "14:F_xx 15:F_xy 16:F_yx 17:F_yy "
          "18:eps_xx 19:eps_xy 20:eps_yx 21:eps_yy "
          "22:L_xx 23:L_xy 24:L_yx 25:L_yy "
          "26:rho 27:vol 28:mass 29:plastic\n";

  for (size_t p = 0; p < session->Conf.MP.size(); p++) {
    const MaterialPoint &MP = session->Conf.MP[p];
    if (MP.pos.x < xmin || MP.pos.x > xmax || MP.pos.y < ymin || MP.pos.y > ymax) { continue; }

    const ProcessedDataMP &D = session->Data[p];

    file << MP.nb << ' ' << MP.groupNb << ' ' << D.pos.x << ' ' << D.pos.y << ' ' << D.vel.x << ' ' << D.vel.y << ' '
         << D.stress.xx << ' ' << D.stress.xy << ' ' << D.stress.yx << ' ' << D.stress.yy << ' ' << D.outOfPlaneStress << ' '
         << D.outOfPlaneShearXZ << ' ' << D.outOfPlaneShearYZ
         << ' ' << D.strain.xx << ' ' << D.strain.xy << ' ' << D.strain.yx << ' ' << D.strain.yy << ' ' << MP.strain.xx
         << ' ' << MP.strain.xy << ' ' << MP.strain.yx << ' ' << MP.strain.yy << ' ' << D.velGrad.xx << ' '
         << D.velGrad.xy << ' ' << D.velGrad.yx << ' ' << D.velGrad.yy << ' ' << D.rho << ' ' << MP.vol << ' '
         << MP.mass << ' ' << (MP.plastic ? 1 : 0) << '\n';
  }
}
