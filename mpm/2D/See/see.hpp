#ifndef SEE_CONF_HPP
#define SEE_CONF_HPP

#ifndef GL_SILENCE_DEPRECATION
#define GL_SILENCE_DEPRECATION
#endif

#ifdef __APPLE__
#include <GL/freeglut.h>
#else
#include <GL/freeglut.h>
#include <GL/freeglut_std.h>
#include <GL/glut.h>
#endif

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>

#include "ColorTable.hpp"
#include "fileTool.hpp"
#include "geoTool.hpp"
#include "glTools.hpp"
#include "message.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

struct AABB {
  double xmin;
  double xmax;
  double ymin;
  double ymax;
};

struct additionalData {
  double MP_x;
  double MP_y;
  double NInt;
  double NB;
  double TF;
  double FF;
  double Rmean;
  double Vmean;
  double VelMean;
  double VelMin;
  double VelMax;
  double VelVar;
  double Vsolid;
  double Vcell;
  double h_xx;
  double h_xy;
  double h_yx;
  double h_yy;
  double ReducedPartDistMean;
};

MPMbox Conf;
std::vector<additionalData> ADsREF;
std::vector<additionalData> ADs;
std::vector<ProcessedDataMP> SmoothedData;
std::vector<MaterialPoint> MPREF;
std::vector<colorRGBA> precompColors;
int confNum = 1;

AABB worldBox;

int main_window;

// flags
int show_background        = 0;
int show_MPs               = 1;
int show_grid              = 1;
int show_obstacles         = 1;
int show_stress_directions = 0;

int show_node_dofs                = 0;
int show_node_velocity_directions = 0;

int MP_deformed_shape = 0;
int MP_contour        = 1;

int color_option = 0;
ColorTable colorTable;
glColorBar colorBar;

int width      = 800;
int height     = 600;
float wh_ratio = (float)width / (float)height;

glTextZone textZone(1, &width, &height);

// Miscellaneous global variables
enum MouseMode { NOTHING, ROTATION, ZOOM, PAN } mouse_mode = NOTHING;
int mouse_start[2];

// Drawing functions
void setColor(int i, float alpha = 1.0f);
void drawGrid();
void drawMPs();
void drawObstacles();
void drawStressDirections();

// Callback functions
void keyboard(unsigned char Key, int x, int y);
void mouse(int button, int state, int x, int y);
void motion(int x, int y);
void display();
void reshape(int x, int y);

// menu
void menu(int num);
void buildMenu();

// Helper functions
void precomputeColors(int n);
void printHelp();
void fit_view();
bool try_to_readConf(int num, MPMbox &CF, int &OKNum);
void updateTextLine();
void readConf(const char *file_name, const char *co_file_name, MPMbox &CF);
void readAdditionalData(const char *fileName);
int screenshot(const char *filename);

#endif /* end of include guard: SEE_CONF_HPP */
