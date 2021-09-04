#ifndef SEE_HPP
#define SEE_HPP

#include <GL/freeglut.h>

/*
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
*/

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>

#include "ColorTable.hpp"
#include "PBC3D.hpp"
#include "graphGL.hpp"

PBC3Dbox box;
int confNum = 0;

int main_window;

// flags
int show_background = 1;
int show_particles = 1;
int show_velocities = 0;
int show_cell = 1;
int show_ghosts = 0;
int show_slice = 0;
int show_forces = 0;

GLfloat alpha_particles = 1.0f;
GLfloat alpha_ghosts = 0.3f;

double ghost_width = 0.2;
double arrowSize = 0.0005;
double arrowAngle = 0.7;

double forceTubeFactor = 1.0;

double radiusMin;
double radiusMax;
double radiusMean;

int width = 800;
int height = 800;
float wh_ratio = (float)width / (float)height;

// Miscellaneous global variables
enum MouseMode { NOTHING, ROTATION, ZOOM, PAN } mouse_mode = NOTHING;
int display_mode = 0; // sample or slice rotation
int mouse_start[2];
float view_angle;
float znear;
float zfar;
GLfloat Rot_Matrix[16];
GLfloat max_length;

vec3r eye;
vec3r center;
vec3r up;

vec3r slice_pos;
vec3r slice_norm;
double slice_width;

ColorTable ParticleColorTable;
ColorTable ForceColorTable;
ColorTable DamCol; 

// Data for drawing spheres
#define X .525731112119133606
#define Z .850650808352039932

static GLfloat vdata[12][3] = {{-X, 0.0, Z}, {X, 0.0, Z},   {-X, 0.0, -Z}, {X, 0.0, -Z}, {0.0, Z, X},  {0.0, Z, -X},
                               {0.0, -Z, X}, {0.0, -Z, -X}, {Z, X, 0.0},   {-Z, X, 0.0}, {Z, -X, 0.0}, {-Z, -X, 0.0}};
static GLuint tindices[20][3] = {{0, 4, 1}, {0, 9, 4},  {9, 5, 4},  {4, 5, 8},  {4, 8, 1},  {8, 10, 1}, {8, 3, 10},
                                 {5, 3, 8}, {5, 2, 3},  {2, 7, 3},  {7, 10, 3}, {7, 6, 10}, {7, 11, 6}, {11, 0, 6},
                                 {0, 1, 6}, {6, 1, 10}, {9, 0, 11}, {9, 11, 2}, {9, 2, 5},  {7, 2, 11}};

struct GLColorRGBA {
  GLfloat r, g, b, a;
  GLColorRGBA() : r(0.5f), g(0.5f), b(0.5f), a(1.0f) {}
  GLColorRGBA(GLfloat R, GLfloat G, GLfloat B, GLfloat A) : r(R), g(G), b(B), a(A) {}
};

// Drawing functions
void drawsphere(int ndiv, float radius);
void drawtri(GLfloat *a, GLfloat *b, GLfloat *c, int div, float r);
void normalize(GLfloat *a);

void clear_background();
void drawPeriodicCell();
void drawSlice();
void drawArrow(vec3r &orig, vec3r &arrow);
void drawTube(vec3r &orig, vec3r &arrow, double diam);
void drawForces();
void drawVelocities();
void drawParticles();
void drawGhosts();

// Colorizer functions
GLColorRGBA colorParticleNone(int i);
GLColorRGBA colorParticleVelocityMagnitude(int i);

GLColorRGBA colorForceNone(int i);

// Plotting functions
void plot_fnft();
void plot_qp();

// Pointers of functions
std::function<GLColorRGBA(int)> colorParticle = colorParticleNone;
std::function<GLColorRGBA(int)> colorForce = colorForceNone;

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
void quat2GLMatrix(quat &q, GLfloat *pMatrix);
vec3r rotatePoint(vec3r const &p, vec3r const &center, vec3r const &axis, double theta);
void adjust_clipping_plans();
void fit_view();
bool fileExists(const char *fileName);
void try_to_readConf(int num);
void export_sample();
void add_ghost_pos(int i, double mn, double mx, std::vector<vec3r> &lst);
bool inSlice(vec3r &pos);

#endif /* end of include guard: SEE_HPP */
