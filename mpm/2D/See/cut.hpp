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


int confNum;
double xcut;

void try_to_readConf(int num, MPMbox& CF);
