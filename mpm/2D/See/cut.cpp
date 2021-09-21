#include "cut.hpp"

#include <typeinfo>

#include "Obstacles/Circle.hpp"
#include "Obstacles/Line.hpp"


void try_to_readConf(int num, MPMbox& CF) {
  char file_name[256];
  sprintf(file_name, "conf%d.txt", num);
  std::cout << "Read " << file_name << std::endl;
  CF.clean();
  CF.read(file_name);
  CF.CutProcess(SmoothedData);
}



int main(int argc, char* argv[]) {

  confNum = (argc>1) ? atoi(argv[1]) : 0;
  xcut    = (argc>2) ? atof(argv[2]) : 0.0;
  result_folder= "pressure";
  std::cout << "Current Configuration: ";
  fileTool::create_folder(result_folder);
  try_to_readConf(confNum, Conf);
  char name[256];

 if(argc>2){
    sprintf(name, "./%s/c%d_x_%1.2e.txt", result_folder.c_str(), confNum,xcut);
    file.open(name);
    file << "# MP.x MP.y sigma_xx sigma_yy sigma_xy sigma_yx v_x v_y" << std::endl;
    for (size_t i = 0; i < Conf.MP.size(); i++) {
      if(SmoothedData[i].corner[0].x<=xcut && SmoothedData[i].corner[2].x>=xcut){
       file << SmoothedData[i].pos.x     << " "
            << SmoothedData[i].pos.y     << " "
            << SmoothedData[i].stress.xx << " "
            << SmoothedData[i].stress.yy << " "
            << SmoothedData[i].stress.xy << " "
            << SmoothedData[i].stress.yx << " "
            << SmoothedData[i].vel.x     << " "
            << SmoothedData[i].vel.y     << " "
            << SmoothedData[i].strain.xx << " "
            << SmoothedData[i].strain.yy << " "
            << SmoothedData[i].strain.xy << " "
            << SmoothedData[i].strain.yx << " "
            << SmoothedData[i].rho       << std::endl;
       }
    }
    file.close();
  }
  else{
    sprintf(name, "./%s/c%d_2D.txt", result_folder.c_str(), confNum);
    file.open(name);
    file << "# MP.x MP.y sigma_xx sigma_yy sigma_xy sigma_yx v_x v_y" << std::endl;
    for (size_t i = 0; i < Conf.MP.size(); i++) {
          file << 0.5*(SmoothedData[i].corner[0].x+SmoothedData[i].corner[2].x) << " " 
            << 0.5*(SmoothedData[i].corner[0].y+SmoothedData[i].corner[2].y) << " "
            << SmoothedData[i].stress.xx << " "
            << SmoothedData[i].stress.yy << " "
            << SmoothedData[i].stress.xy << " "
            << SmoothedData[i].stress.yx << " "
            << SmoothedData[i].vel.x << " "
            << SmoothedData[i].vel.y << std::endl;
     }
  }
  file.close();
  return 0;
}
