#include "see.hpp"

void printHelp() {
  using namespace std;
  cout << endl;
  cout << "+         load next configuration file" << endl;
  cout << "-         load previous configuration file" << endl;
  cout << "=         fit the view" << endl;
  cout << "q         quit" << endl;
  // cout << "" << endl;
  cout << endl;
}

void printInfo() {
  using namespace std;

  cout << "\nCurrent Conf = " << confNum << "\n\n";
  /*
  cout << "Cell.h: " << Conf.Cell.h << '\n';
  cout << "Cell.vh: " << Conf.Cell.vh << '\n';
  cout << "Cell.mass: " << Conf.Cell.mass << '\n';

  cout << "Vsolid: " << Conf.Vsolid << '\n';
  cout << "Vmin: " << Conf.Vmin << '\n';
  cout << "Vmax: " << Conf.Vmax << '\n';
  cout << "Vmean: " << Conf.Vmean << '\n';

  cout << "Rmin: " << Conf.Rmin << '\n';
  cout << "Rmax: " << Conf.Rmax << '\n';
  cout << "Rmean: " << Conf.Rmean << '\n';

  cout << "FnMin: " << Conf.FnMin << '\n';
  cout << "FnMax: " << Conf.FnMax << '\n';
  cout << "FnMean: " << Conf.FnMean << '\n';

  cout << "VelMin: " << Conf.VelMin << '\n';
  cout << "VelMax: " << Conf.VelMax << '\n';
  cout << "VelMean: " << Conf.VelMean << '\n';
  */
}

void keyboard(unsigned char Key, int /*x*/, int /*y*/) {
  switch (Key) {

    case '0': {
      color_option = 0;
    } break;

    case '1': {
      color_option = 1;
      double vmax = 0.0;
      for (size_t i = 0 ; i < Conf.MP.size(); i++) {
        double vel = norm(Conf.MP[i].vel);
        if (vel > vmax) vmax = vel;
      }
      colorTable.setMinMax(0.0, vmax);
      colorTable.setTableID(7);
      colorTable.Rebuild();
      
    } break;

    case 'i': {
      printInfo();
    } break;

    case 'g': {
      show_grid = 1 - show_grid;
    } break;

    case 'q': {
      exit(0);
    } break;

    case '-': {
      if (confNum > 0) try_to_readConf(confNum - 1, Conf, confNum);
    } break;

    case '+': {
      try_to_readConf(confNum + 1, Conf, confNum);
    } break;

    case '=': {
      fit_view();
      reshape(width, height);
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

void motion(int x, int y) {

  if (mouse_mode == NOTHING) return;

  double dx = (double)(x - mouse_start[0]) / (double)width;
  double dy = (double)(y - mouse_start[1]) / (double)height;

  switch (mouse_mode) {

    case ZOOM: {
      double ddy = (worldBox.ymax - worldBox.ymin) * dy;
      double ddx = (worldBox.xmax - worldBox.xmin) * dy;
      worldBox.xmin -= ddx;
      worldBox.xmax += ddx;
      worldBox.ymin -= ddy;
      worldBox.ymax += ddy;
    } break;

    case PAN: {
      double ddx = (worldBox.xmax - worldBox.xmin) * dx;
      double ddy = (worldBox.ymax - worldBox.ymin) * dy;
      worldBox.xmin -= ddx;
      worldBox.xmax -= ddx;
      worldBox.ymin += ddy;
      worldBox.ymax += ddy;
    } break;

    default:
      break;
  }
  mouse_start[0] = x;
  mouse_start[1] = y;

  reshape(width, height);
  display();
}

void display() {
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glShadeModel(GL_SMOOTH);
  glEnable(GL_DEPTH_TEST);

  if (show_grid == 1) drawGrid();

  drawMPs();

  glFlush();
  glutSwapBuffers();
}

void fit_view() {
  worldBox.xmin = 0.0;
  worldBox.ymin = 0.0;
  worldBox.xmax = 0.0;
  worldBox.ymax = 0.0;
  for (size_t i = 0; i < Conf.nodes.size(); i++) {
    if (Conf.nodes[i].pos.x > worldBox.xmax) worldBox.xmax = Conf.nodes[i].pos.x;
    if (Conf.nodes[i].pos.y > worldBox.ymax) worldBox.ymax = Conf.nodes[i].pos.y;
  }
}

void reshape(int w, int h) {
  width = w;
  height = h;
  GLfloat aspect = (GLfloat)width / (GLfloat)height;
  double left = worldBox.xmin;
  double right = worldBox.xmax;
  double bottom = worldBox.ymin;
  double top = worldBox.ymax;
  double worldW = right - left;
  double worldH = top - bottom;
  double dW = 0.2 * worldW;
  double dH = 0.2 * worldH;
  left -= dW;
  right += dW;
  top += dH;
  bottom -= dH;
  worldW = right - left;
  worldH = top - bottom;
  if (worldW >= worldH) {
    worldH = worldW / aspect;
    top = 0.5 * (bottom + top + worldH);
    bottom = top - worldH;
  } else {
    worldW = worldH * aspect;
    right = 0.5 * (left + right + worldW);
    left = right - worldW;
  }

  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluOrtho2D(left, right, bottom, top);

  glutPostRedisplay();
}

void drawGrid() {
  glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
  glLineWidth(1.0f);

  std::vector<node>& nodes = Conf.nodes;
  int* I;
  for (size_t e = 0; e < Conf.Elem.size(); e++) {
    I = &(Conf.Elem[e].I[0]);
    glBegin(GL_LINE_LOOP);
    for (int r = 0; r < 4; r++) glVertex2f(nodes[I[r]].pos.x, nodes[I[r]].pos.y);
    glEnd();
  }
}

void setColor(int i) {
  switch (color_option) {

    case 0: {
      glColor4f(0.8f, 0.8f, 0.9f, 1.0f);
    } break;

    case 1: {
      colorRGBA col;
      double vel = norm(Conf.MP[i].vel);
      colorTable.getRGB(vel, &col);
      glColor4f(col.r / 255.0, col.g / 255.0, col.b / 255.0, 1.0f);
    } break;

    default: {
      glColor4f(0.8f, 0.8f, 0.9f, 1.0f);
    } break;
  }
}

void drawMPs() {
  glLineWidth(1.0f);

  for (size_t i = 0; i < Conf.MP.size(); ++i) {
    
    double xc = Conf.MP[i].pos.x;
    double yc = Conf.MP[i].pos.y;
    double R = 0.5* Conf.MP[i].size;

    setColor(i);
    glBegin(GL_POLYGON);
    for (double angle = 0.0; angle < 2.0 * M_PI; angle += 0.05 * M_PI) {
      glVertex2f(xc + R * cos(angle), yc + R * sin(angle));
    }
    glEnd();

    
    glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
    glBegin(GL_LINE_LOOP);
    for (double angle = 0.0; angle < 2.0 * M_PI; angle += 0.05 * M_PI) {
      glVertex2f(xc + R * cos(angle), yc + R * sin(angle));
    }
    glEnd();
    
  }
}

// void drawContacts() {}
/*
void drawForces() {
  if (Conf.Interactions.empty()) return;

  double fnMax = Conf.Interactions[0].fn;
  for (size_t k = 1; k < Conf.Interactions.size(); ++k) {
    if (Conf.Interactions[k].fn > fnMax) fnMax = Conf.Interactions[k].fn;
  }

  glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
  for (size_t k = 0; k < Conf.Interactions.size(); ++k) {
    size_t i = Conf.Interactions[k].i;
    size_t j = Conf.Interactions[k].j;
    vec2r sij = Conf.Particles[j].pos - Conf.Particles[i].pos;
    sij.x -= floor(sij.x + 0.5);
    sij.y -= floor(sij.y + 0.5);
    vec2r posi = Conf.Cell.h * Conf.Particles[i].pos;
    vec2r posj = posi + Conf.Cell.h * sij;
    vec2r n = posj - posi;
    n.normalize();
    vec2r t(-n.y, n.x);
    double w = Conf.Interactions[k].fn / fnMax * 0.5 * Conf.Rmean;

    glBegin(GL_POLYGON);
    glVertex2f(posi.x + w * (t.x), posi.y + w * (t.y));
    glVertex2f(posj.x + w * (t.x), posj.y + w * (t.y));
    glVertex2f(posj.x + w * (-t.x), posj.y + w * (-t.y));
    glVertex2f(posi.x + w * (-t.x), posi.y + w * (-t.y));
    glEnd();
  }
}
*/

/// Robust and portable function to test if a file exists
bool fileExists(const char* fileName) {
  std::fstream fin;
  fin.open(fileName, std::ios::in);
  if (fin.is_open()) {
    fin.close();
    return true;
  }
  fin.close();
  return false;
}

void try_to_readConf(int num, MPMbox& CF, int& OKNum) {
  char file_name[256];
  sprintf(file_name, "conf%d.txt", num);
  if (fileExists(file_name)) {
    std::cout << "Read " << file_name << std::endl;
    OKNum = num;
    CF.clean();
    CF.read(file_name);
  } else
    std::cout << file_name << " does not exist" << std::endl;
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
  /*
  int submenu1 = glutCreateMenu(menu);  // Particle Colors
  glutAddMenuEntry("None", 100);
  glutAddMenuEntry("Velocity Magnitude", 101);
  glutAddMenuEntry("Sum of Normal Contact Forces", 102);

  int submenu2 = glutCreateMenu(menu);  // Force Colors
  glutAddMenuEntry("None", 200);
  glutAddMenuEntry("Magnitude", 201);
  */
  
  glutCreateMenu(menu);  // Main menu
  //glutAddSubMenu("Particle Colors", submenu1);
  //glutAddSubMenu("Force Colors", submenu2);
  //glutAddSubMenu("Velocity Colors", submenu2);
  glutAddMenuEntry("Quit", 0);
}

// =====================================================================
// Main function
// =====================================================================

int main(int argc, char* argv[]) {

  if (argc == 1) {
    confNum = 0;
  } else if (argc == 2) {
    confNum = atoi(argv[1]);
  }

  std::cout << "Current Configuration: ";
  try_to_readConf(confNum, Conf, confNum);

  // ==== Init GLUT and create window
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA);
  glutInitWindowPosition(50, 50);
  glutInitWindowSize(width, height);
  main_window = glutCreateWindow("CONF VISUALIZER");

  // ==== Register callbacks
  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);

  // ==== Menu
  buildMenu();
  glutAttachMenu(GLUT_RIGHT_BUTTON);

  mouse_mode = NOTHING;

  glEnable(GL_BLEND);
  glBlendEquation(GL_FUNC_ADD);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // ==== Enter GLUT event processing cycle
  fit_view();
  glutMainLoop();
  return 0;
}
