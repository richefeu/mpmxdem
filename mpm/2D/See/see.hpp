#ifndef SEE_CONF_HPP
#define SEE_CONF_HPP

//#include <GL/freeglut.h>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

#include "ColorTable.hpp"

struct AABB {
  double xmin;
  double xmax;
  double ymin;
  double ymax;
};

MPMbox Conf;
std::vector<ProcessedDataMP> SmoothedData;
int confNum = 1;

AABB worldBox;

int main_window;

// flags
int show_background = 1;
int show_MPs = 1;
int show_grid = 1;
int show_obstacles = 1;

int MP_deformed_shape = 0;

int color_option = 0;
ColorTable colorTable;

int width = 800;
int height = 600;
float wh_ratio = (float)width / (float)height;

// Miscellaneous global variables
enum MouseMode { NOTHING, ROTATION, ZOOM, PAN } mouse_mode = NOTHING;
int mouse_start[2];

// Drawing functions
void setColor(int i);
void drawGrid();
void drawMPs();
void drawObstacles();

// Callback functions
void keyboard(unsigned char Key, int x, int y);
void mouse(int button, int state, int x, int y);
void motion(int x, int y);
void display();
void reshape(int x, int y);
void menu(int num);

// Helper functions
void buildMenu();
void printHelp();
void fit_view();
bool fileExists(const char *fileName);
void try_to_readConf(int num, MPMbox &CF, int &OKNum);

#endif /* end of include guard: SEE_CONF_HPP */