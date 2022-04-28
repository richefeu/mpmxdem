#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

MPMbox Conf;
std::vector<ProcessedDataMP> SmoothedData;
std::string result_folder;
std::ofstream file;
std::ofstream file_micro;


int confNum;
double xcut;
double aniso;
vec2r d1,d2;
void try_to_readConf(int num, MPMbox& CF);
