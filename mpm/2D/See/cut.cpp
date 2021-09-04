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
  CF.postProcess(SmoothedData);
}



int main(int argc, char* argv[]) {

  confNum      = atoi(argv[1]);
  xcut         = atof(argv[2]);
  result_folder= "pressure";
  std::cout << "Current Configuration: ";
  char name[256];
  sprintf(name, "./%s/c%d_x_%1.2e.txt", result_folder.c_str(), confNum,xcut);
  fileTool::create_folder(result_folder);
  std::ofstream file(name);
  try_to_readConf(confNum, Conf);
  file << "# MP.x MP.y p(x,y)" << std::endl;


 for (size_t i = 0; i < Conf.MP.size(); i++) {
     if(SmoothedData[i].corner[0].x<=xcut && SmoothedData[i].corner[2].x>=xcut){
      file << 0.5*(SmoothedData[i].corner[0].x+SmoothedData[i].corner[2].x) << " " 
           << 0.5*(SmoothedData[i].corner[0].y+SmoothedData[i].corner[2].y) << " "
           <<0.5 * (SmoothedData[i].stress.xx + SmoothedData[i].stress.yy) 
           << std::endl;
      }
   }
   file.close();
   return 0;
}
