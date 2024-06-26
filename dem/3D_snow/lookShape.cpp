#include "lookShape.hpp"

#include "fileTool.hpp"
#include "glutTools.hpp"

void printHelp() {
  switch2D::go(width, height);

  glColor4f(1.0f, 1.0f, 1.0f, 0.6f);
  glBegin(GL_QUADS);
  int nbLines = 9;  // update this value when a line is added
  int by = height - nbLines * 15 - 3;
  glVertex2i(0, height);
  glVertex2i(width, height);
  glVertex2i(width, by);
  glVertex2i(0, by);
  glEnd();

  glColor3i(0, 0, 0);
  int dhline = -15;
  int hline = height;
#define _nextLine_ (hline += dhline)
  glText::print(GLUT_BITMAP_8_BY_13, 15, _nextLine_, "[A][a]   Tune alpha (transparency)");
  glText::print(GLUT_BITMAP_8_BY_13, 15, _nextLine_, "[b]      Background (color gradient) on/off");
  glText::print(GLUT_BITMAP_8_BY_13, 15, _nextLine_, "[c]      Compute mass properties of the current shape");
  glText::print(GLUT_BITMAP_8_BY_13, 15, _nextLine_, "[C]      Compute mass properties of ALL shapes");
  glText::print(GLUT_BITMAP_8_BY_13, 15, _nextLine_, "[e]      print extents of the current shape");
  glText::print(GLUT_BITMAP_8_BY_13, 15, _nextLine_, "[h]      Show this help");
  glText::print(GLUT_BITMAP_8_BY_13, 15, _nextLine_, "[q]      Quit");
  glText::print(GLUT_BITMAP_8_BY_13, 15, _nextLine_, "[s]      Save the shape library");
  glText::print(GLUT_BITMAP_8_BY_13, 15, _nextLine_, "[+][-]   Navigate through the shapes");
#undef _nextLine_

  switch2D::back();
}

void keyboard(unsigned char Key, int x, int y) {
  switch (Key) {

    case 'A':
      if (alpha < 1.0f) {
        alpha += 0.05f;
        if (alpha > 1.0f) alpha = 1.0f;
      }
      break;
    case 'a':
      if (alpha >= 0.1f) alpha -= 0.05f;
      break;

    case 'b':
      show_background = 1 - show_background;
      break;

    case 'c':
      Shapes[ishape].massProperties();
      fit_view();
      break;

    case 'C': {
      for (size_t i = 0; i < Shapes.size(); i++) {
        Shapes[i].massProperties();
      }
      fit_view();
    } break;

    case 'e': {
      std::cout << "extents: " << Shapes[ishape].obb.extent[0] << " " << Shapes[ishape].obb.extent[1] << " "
                << Shapes[ishape].obb.extent[2] << std::endl;
    } break;

    case 'h':
      show_help = 1 - show_help;
      break;

    case 'f': {
      Shapes[ishape].fitObb();
    } break;


    case 'P': {
      prepro("clumps.txt");
    } break;
    
    case 'q':
      exit(0);
      break;

    case 's':
      saveShapeLib(shapeFileName.c_str());
      break;

    case '-': {
      if (ishape > 0) ishape--;
      fit_view();
    } break;

    case '+': {
      ishape++;
      if (ishape >= Shapes.size()) ishape = Shapes.size() - 1;
      fit_view();
    } break;
  };

  glutPostRedisplay();
}

void mouse(int button, int state, int x, int y) {
  if (state == GLUT_UP) {
    mouse_mode = NOTHING;
    display();
  } else if (state == GLUT_DOWN) {
    mouse_start[0] = x;
    mouse_start[1] = y;
    switch (button) {
      case GLUT_LEFT_BUTTON:
        if (glutGetModifiers() == GLUT_ACTIVE_SHIFT)
          mouse_mode = PAN;
        else
          mouse_mode = ROTATION;
        break;
      case GLUT_MIDDLE_BUTTON:
        mouse_mode = ZOOM;
        break;
    }
  }
}

vec3r rotatePoint(vec3r const& p, vec3r const& center, vec3r const& axis, double theta) {
  double const c = cos(theta), s = sin(theta);
  double const C = 1.0 - c;
  vec3r tmp = p - center;
  return center + vec3r(tmp[0] * (axis[0] * axis[0] * C + c) + tmp[1] * (axis[0] * axis[1] * C - axis[2] * s) +
                            tmp[2] * (axis[0] * axis[2] * C + axis[1] * s),
                        tmp[0] * (axis[1] * axis[0] * C + axis[2] * s) + tmp[1] * (axis[1] * axis[1] * C + c) +
                            tmp[2] * (axis[1] * axis[2] * C - axis[0] * s),
                        tmp[0] * (axis[2] * axis[0] * C - axis[1] * s) +
                            tmp[1] * (axis[2] * axis[1] * C + axis[0] * s) + tmp[2] * (axis[2] * axis[2] * C + c));
}

void motion(int x, int y) {
  if (mouse_mode == NOTHING) return;

  double dx = (double)(x - mouse_start[0]) / (double)width;
  double dy = (double)(y - mouse_start[1]) / (double)height;
  double length;
  vec3r axis;

  switch (mouse_mode) {

    case ROTATION:
      axis = (cross(up, center - eye));
      axis.normalize();
      eye = rotatePoint(eye, center, up, -dx * M_PI);
      eye = rotatePoint(eye, center, axis, dy * M_PI);
      up = (rotatePoint((center + up), center, axis, dy * M_PI) - center);
      up.normalize();
      break;

    case ZOOM:
      eye = center + (eye - center) * (dy + 1.0);
      break;

    case PAN:
      length = (eye - center).length() * tan(view_angle * M_PI / 360.0) * 2.0;
      axis = cross(up, center - eye);
      axis.normalize();
      center = center + axis * dx * length * 0.8;
      center = center + up * dy * length;
      break;

    default:
      break;
  }
  mouse_start[0] = x;
  mouse_start[1] = y;

  display();
}

// Les informations ecritent en bas de la fenetre
void drawInfo() {
  switch2D::go(width, height);
  glColor3f(1.0f, 0.388f, 0.278f);  // dark-orange

  glText::print(GLUT_BITMAP_9_BY_15, 10, 10, "Shape %lu/%lu", ishape + 1, Shapes.size());
  glText::print(GLUT_BITMAP_9_BY_15, 10, 25, "nb spheres = %lu", Shapes[ishape].subSpheres.size());
  glText::print(GLUT_BITMAP_9_BY_15, 10, 40, "I/m %g %g %g", Shapes[ishape].I_m[0], Shapes[ishape].I_m[1],
                Shapes[ishape].I_m[2]);

  switch2D::back();
}

void display() {
  glutTools::clearBackground(show_background);
  adjust_clipping_plans();
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  gluLookAt(eye.x, eye.y, eye.z, center.x, center.y, center.z, up.x, up.y, up.z);

  drawFrame();

  glShadeModel(GL_SMOOTH);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);

  drawShape(ishape);

  glColor3f(0.8f, 0.2f, 0.2f);
  glutShape::drawObb(Shapes[ishape].obb);

  drawObbLevel(ishape, 10);

  drawInfo();
  if (show_help) printHelp();

  glFlush();
  glutSwapBuffers();
}

void adjust_clipping_plans() {
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  wh_ratio = (float)width / (float)height;
  double zf = (eye - center).normalize();
  OBB& obb = Shapes[ishape].obb;
  vec3r mx = 2 * (obb.extent[0] * obb.e[0] + obb.extent[1] * obb.e[1] + obb.extent[2] * obb.e[2]);
  max_length = (GLfloat)(2 * norm(mx));
  double znear = zf - 0.5 * max_length;
  double close_dst = 0.1 * zf;
  if (znear < close_dst) znear = close_dst;
  double zfar = zf + max_length;
  gluPerspective(view_angle, wh_ratio, znear, zfar);
  glMatrixMode(GL_MODELVIEW);
}

void fit_view() {
  vec3r dir = (eye - center);
  OBB& obb = Shapes[ishape].obb;
  vec3r diag = 2.0 * (obb.extent[0] * obb.e[0] + obb.extent[1] * obb.e[1] + obb.extent[2] * obb.e[2]);
  dir.normalize();
  center = obb.center;
  GLfloat d = 0.5 * diag.length() / (atan(view_angle * M_PI / 360.0));
  eye = center + d * dir;
}

void reshape(int w, int h) {
  width = w;
  height = h;
  glViewport(0, 0, width, height);

  adjust_clipping_plans();
  glutPostRedisplay();
}

void drawObbLevel(size_t ishp, size_t level) {
  // glDisable(GL_LIGHTING);
  // glColor3f(0.2f, 0.2f, 0.8f);
  //
  // for (size_t i = 0; i < Shapes[ishp].tree.nodes.size(); i++) {
  //   if (Shapes[ishp].tree.nodes[i].level != maxOBBLevel) continue;
  //   glutShape::drawObb(Shapes[ishp].tree.nodes[i].obb);
  // }
}

void drawFrame() {
  OBB& obb = Shapes[ishape].obb;
  vec3r diag = 2 * (obb.extent[0] * obb.e[0] + obb.extent[1] * obb.e[1] + obb.extent[2] * obb.e[2]);
  double len = diag.length() * 0.333;

  glColor3f(1.0f, 0.0f, 0.0f);
  glutShape::drawArrow(vec3r::zero(), len * vec3r::unit_x());
  glColor3f(0.0f, 1.0f, 0.0f);
  glutShape::drawArrow(vec3r::zero(), len * vec3r::unit_y());
  glColor3f(0.0f, 0.0f, 1.0f);
  glutShape::drawArrow(vec3r::zero(), len * vec3r::unit_z());
}

void drawShape(size_t ishp) {
  if (ishp >= Shapes.size()) return;
  if (mouse_mode != NOTHING) return;

  glColor4f(0.666f, 0.729f, 0.09f, alpha);
  size_t nv = Shapes[ishp].subSpheres.size();
  for (size_t s = 0; s < nv; s++) {

    double R = Shapes[ishp].subSpheres[s].radius;
    vec3r pos = Shapes[ishp].subSpheres[s].localPos;
    glPushMatrix();
    glTranslatef(pos.x, pos.y, pos.z);
    glutShape::sphere(R, 3);
    glPopMatrix();
  }
}

int readShapeLib(const char* fileName) {
  if (!fileTool::fileExists(fileName)) {
    std::cout << "Shape Library named '" << fileName << "' has not been found." << std::endl;
    return 0;
  }
  shapeFileName = std::string(fileName);

  std::ifstream is(fileName);

  std::string token;
  is >> token;
  while (is) {
    if (token == "<") {
      Particle S;
      S.readShape(is, 1.0);
      Shapes.push_back(S);
    }
    is >> token;
  }

  std::cout << "Number of Shapes found: " << Shapes.size() << std::endl;

  ishape = 0;
  Shapes[ishape].fitObb();
  OBB& obb = Shapes[ishape].obb;
  center.set(obb.center.x, obb.center.y, obb.center.y);  // where we look at
  eye.set(obb.center.x + obb.extent.x, obb.center.y,
          obb.center.y);  // from where we look
  up.set(0.0, 1.0, 0.0);  // direction (normalized)

  return 1;
}

void saveShapeLib(const char* fileName) {
  std::ofstream os;

  if (fileTool::fileExists(fileName)) {
    std::string newFileName = fileTool::GetFilePath(fileName) + "/mod_" + fileTool::GetFileName(fileName) + "." +
                              fileTool::GetFileExt(fileName);
    std::cout << "save " << newFileName << std::endl;
    os.open(newFileName);
  } else {
    std::cout << "save " << fileName << std::endl;
    os.open(fileName);
  }

  for (size_t s = 0; s < Shapes.size(); s++) {
    Shapes[s].writeShape(os);
  }
}

void prepro(const char* fileName) {

  for (size_t i = 0; i < Shapes.size(); i++) {
    std::cout << "Shape " << i << "/" << Shapes.size() << '\n';
    Shapes[i].massProperties();
  }

  saveShapeLib("shapes.txt");
  
  std::ofstream os(fileName);
  os << "Particles " << Shapes.size() << '\n';
  for (size_t i = 0; i < Shapes.size(); i++) {
    os << Shapes[i].origPos/0.005 << "  0 0 0  0 0 0  " << Shapes[i].Q << "  0 0 0  0 0 0\n";
  }
}

void menu(int num) {
  switch (num) {

    case 0:
      exit(0);
      break;
  };

  glutPostRedisplay();
}

void buildMenu() {
  glutCreateMenu(menu);  // Main menu

  glutAddMenuEntry("Export release config. for DEMbox", 1);
  glutAddMenuEntry("Quit", 0);
}

// =====================================================================
// Main function
// =====================================================================

int main(int argc, char* argv[]) {
  if (argc == 1) {
    if (readShapeLib("shapes.txt") == 0) return 0;
  } else if (argc == 2) {
    if (readShapeLib(argv[1]) == 0) return 0;
  }

  // ==== Init GLUT and create window
  glutInit(&argc, argv);
  glutSetOption(GLUT_MULTISAMPLE, 8);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH | GLUT_MULTISAMPLE);
  glutInitWindowPosition(50, 50);
  glutInitWindowSize(width, height);
  main_window = glutCreateWindow("lookShape for sphere-clumps");

  // ==== Register callbacks
  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);
  // glutSpecialFunc(processSpecialKeys);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);

  // ==== Menu
  buildMenu();
  glutAttachMenu(GLUT_RIGHT_BUTTON);

  // ==== Init the visualizer
  center.set(0.0, 0.0, 0.0);  // where we look at
  eye.set(0.0, 0.0, 1.0);     // from where we look
  up.set(0.0, 1.0, 0.0);      // direction (normalized)

  mouse_mode = NOTHING;
  view_angle = 45.0;
  znear = 0.01;
  zfar = 10.0;

  glDisable(GL_CULL_FACE);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHT1);
  glEnable(GL_COLOR_MATERIAL);

  // Create light components
  GLfloat ambientLight[] = {0.2f, 0.2f, 0.2f, 1.0f};
  GLfloat diffuseLight[] = {0.8f, 0.8f, 0.8, 1.0f};
  GLfloat specularLight[] = {0.5f, 0.5f, 0.5f, 1.0f};
  GLfloat positionLight0[] = {1000000.0f, 1000000.0f, 1000000.0f, 1.0f};
  GLfloat positionLight1[] = {-1000000.0f, -1000000.0f, -1000000.0f, 1.0f};

  // Assign created components to GL_LIGHT0
  glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
  glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);
  glLightfv(GL_LIGHT0, GL_POSITION, positionLight0);

  // Assign created components to GL_LIGHT1
  glLightfv(GL_LIGHT1, GL_AMBIENT, ambientLight);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuseLight);
  glLightfv(GL_LIGHT1, GL_SPECULAR, specularLight);
  glLightfv(GL_LIGHT1, GL_POSITION, positionLight1);

  glShadeModel(GL_SMOOTH);
  glEnable(GL_POINT_SMOOTH);
  glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);

  glEnable(GL_BLEND);
  glBlendEquation(GL_FUNC_ADD);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);

  // ==== Enter GLUT event processing cycle
  adjust_clipping_plans();
  fit_view();
  glutMainLoop();
  return 0;
}
