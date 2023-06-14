#include "see.hpp"

#include <limits>
#include <typeinfo>

#include "Obstacles/Circle.hpp"
#include "Obstacles/Line.hpp"

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
}

void keyboard(unsigned char Key, int /*x*/, int /*y*/) {
  switch (Key) {

    case '0': {
      color_option = 0;
      precomputeColors();
    } break;

    case '1': {
      color_option = 1;
      colorBar.setTitle("velocity magnitude");
      float vmax = 0.0f;
      float vmin = std::numeric_limits<float>::max();
      for (size_t i = 0; i < Conf.MP.size(); i++) {
        float vel = (float)norm(SmoothedData[i].vel);
        if (vel > vmax) vmax = vel;
        if (vel < vmax) vmin = vel;
      }
      colorTable.setMinMax(0.0f, vmax);
      colorTable.setTableID(3);
      colorTable.Rebuild();
      precomputeColors();
      std::cout << "MP colored by velocity magnitude (vmin = " << vmin << ", vmax = " << vmax << ")\n";
    } break;

    case '2': {
      color_option = 2;
      colorBar.setTitle("pressure");
      float pmax = -std::numeric_limits<float>::max();
      float pmin = std::numeric_limits<float>::max();
      for (size_t i = 0; i < Conf.MP.size(); i++) {
        float p = 0.5f * (float)(SmoothedData[i].stress.xx + SmoothedData[i].stress.yy);
        if (p > pmax) pmax = p;
        if (p < pmin) pmin = p;
      }
      // pmin=-10000;
      // pmax=10000;
      colorTable.setMinMax(pmin, pmax);
      // colorTable.setMinMax(-1e4, 1e3);
      colorTable.setTableID(3);
      colorTable.Rebuild();
      precomputeColors();
      std::cout << "MP colored by pressure (pmin = " << pmin << ", pmax = " << pmax << ")\n";
    } break;

    case '3': {
      color_option = 3;
      colorBar.setTitle("density");
      float rhomax = 0.0f;
      float rhomin = std::numeric_limits<float>::max();
      for (size_t i = 0; i < Conf.MP.size(); i++) {
        float rho = (float)SmoothedData[i].rho;
        if (rho > rhomax) rhomax = rho;
        if (rho < rhomin) rhomin = rho;
      }
      colorTable.setMinMax(rhomin, rhomax);
      colorTable.setTableID(3);
      colorTable.Rebuild();
      precomputeColors();
      std::cout << "MP colored by density (rhomin = " << rhomin << ", rhomax = " << rhomax << ")\n";
    } break;

    case '4': {
      color_option = 4;
      colorBar.setTitle("sig_yy");
      float pmax = -std::numeric_limits<float>::max();
      float pmin = std::numeric_limits<float>::max();
      for (size_t i = 0; i < Conf.MP.size(); i++) {
        float p = (float)SmoothedData[i].stress.yy;
        if (p > pmax) pmax = p;
        if (p < pmin) pmin = p;
      }
      colorTable.setMinMax(pmin, pmax);
      colorTable.setTableID(3);
      colorTable.Rebuild();
      precomputeColors();
      std::cout << "MP colored by sig_yy (s_yy_min = " << pmin << ", s_yy_max = " << pmax << ")\n";
    } break;

    case '5': {
      if (ADs.empty()) break;
      color_option = 5;
      colorBar.setTitle("DEM-cell damage");
      float Dmax = 0.0f;
      for (size_t i = 0; i < ADs.size(); i++) {
        if (ADsREF[i].NB == 0.0) continue;
        float D = 1.0f - (float)ADs[i].NB / (float)ADsREF[i].NB;
        if (D > Dmax) Dmax = D;
      }
      colorTable.setMinMax(0.0f, Dmax);
      colorTable.setTableID(6);
      colorTable.Rebuild();
      precomputeColors();
      std::cout << "MP colored by DEM-cell damage [0, " << Dmax << "] \n";
    } break;

    case '6': {
      color_option = 6;
      colorBar.setTitle("deps_q");
      float depsqmax = -std::numeric_limits<float>::max();
      float depsqmin = std::numeric_limits<float>::max();
      for (size_t i = 0; i < Conf.MP.size(); i++) {
        float d1 = (float)(SmoothedData[i].velGrad.xx - SmoothedData[i].velGrad.yy);
        float d2 = (float)(SmoothedData[i].velGrad.xy + SmoothedData[i].velGrad.yx);
        float depsq = (float)sqrt(d1 * d1 + d2 * d2);
        if (depsq > depsqmax) depsqmax = depsq;
        if (depsq < depsqmin) depsqmin = depsq;
      }
      colorTable.setMinMax(depsqmin, depsqmax);
      colorTable.setTableID(3);
      colorTable.Rebuild();
      precomputeColors();
      std::cout << "MP colored by deps_q (deps_q_min = " << depsqmin << ", deps_q_max = " << depsqmax << ")\n";
    } break;
    case '7': {
      color_option = 7;
      colorBar.setTitle("volume variation");
      float volvarmax = -std::numeric_limits<float>::max();
      float volvarmin = std::numeric_limits<float>::max();
      for (size_t i = 0; i < Conf.MP.size(); i++) {
        if (fabs(MPREF[i].F.xx * MPREF[i].F.yy - MPREF[i].F.xy * MPREF[i].F.yx) == 0) continue;
        float volvar = (float)(fabs(Conf.MP[i].F.xx * Conf.MP[i].F.yy - Conf.MP[i].F.xy * Conf.MP[i].F.yx) /
                               fabs(MPREF[i].F.xx * MPREF[i].F.yy - MPREF[i].F.xy * MPREF[i].F.yx)) -
                       1.0f;
        if (volvar > volvarmax) volvarmax = volvar;
        if (volvar < volvarmin) volvarmin = volvar;
      }
      colorTable.setMinMax(volvarmin, volvarmax);
      colorTable.setTableID(3);
      colorTable.Rebuild();
      precomputeColors();
      std::cout << "MP colored by volvar (volvar_min = " << volvarmin << ", volvar_max = " << volvarmax << ")\n";
    } break;
    case '8': {
      color_option = 8;
      colorBar.setTitle("fx");
      float fxmax = -std::numeric_limits<float>::max();
      float fxmin = std::numeric_limits<float>::max();
      for (size_t i = 0; i < Conf.MP.size(); i++) {
        float fx = (float)Conf.MP[i].contactf.x;
        if (fx > fxmax) fxmax = fx;
        if (fx < fxmin) fxmin = fx;
      }
      colorTable.setMinMax(fxmin, fxmax);
      colorTable.setTableID(2);
      colorTable.Rebuild();
      precomputeColors();
      std::cout << "MP colored by fx (fx_min = " << fxmin << ", fx_max = " << fxmax << ")\n";
    } break;
    case '9': {
      if (ADs.empty()) break;
      color_option = 9;
      colorBar.setTitle("inertial number");
      float Imax = -std::numeric_limits<float>::max();
      float Imin = std::numeric_limits<float>::max();
      for (size_t i = 0; i < Conf.MP.size(); i++) {
        float d1 = (float)(SmoothedData[i].velGrad.xx - SmoothedData[i].velGrad.yy);
        float d2 = (float)(SmoothedData[i].velGrad.xy + SmoothedData[i].velGrad.yx);
        float pres = 0.5f * (float)(SmoothedData[i].stress.xx + SmoothedData[i].stress.yy);
        float I =
            (float)((2 * ADs[i].Rmean * sqrt(d1 * d1 + d2 * d2)) / sqrt((abs(pres) + 1e-6) / SmoothedData[i].rho));
        if (I > Imax) Imax = I;
        if (I < Imin) Imin = I;
      }
      colorTable.setMinMax(Imin, Imax);
      colorTable.setTableID(3);
      colorTable.Rebuild();
      precomputeColors();
      std::cout << "MP colored by I (I_min = " << Imin << ", I_max = " << Imax << ")\n";
    } break;

    case 'd': {
      show_node_dofs = 1 - show_node_dofs;
    } break;

    case 'b': {
      show_background = 1 - show_background;
    } break;

    case 'c': {
      MP_contour = 1 - MP_contour;
    } break;

    case 'g': {
      show_grid = 1 - show_grid;
    } break;

    case 'h': {
      printHelp();
    } break;

    case 'i': {
      printInfo();
    } break;

    case 'm': {
      show_MPs = 1 - show_MPs;
    } break;

    case 'n': {
      std::cout << "number: ";
      int num;
      std::cin >> num;
      try_to_readConf(num, Conf, confNum);
    } break;

    case 'q': {
      exit(0);
    } break;

    case 's': {
      MP_deformed_shape = 1 - MP_deformed_shape;
    } break;

    case 'x': {
      show_stress_directions = 1 - show_stress_directions;
    } break;

    case 'z': {
      std::cout << "image saved in 'oneshot.tga'\n";
      screenshot("oneshot.tga");
    } break;
    case 'Z': {
      // be carreful there's no way to stop this loop
      // if the process if too long
      while (try_to_readConf(confNum + 1, Conf, confNum)) {
        char name[256];
        snprintf(name, 256, "shot%d.tga", confNum);
        display();
        screenshot(name);
      }
      std::cout << "series of images saved in 'shot<n>.tga'\n";
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
      case GLUT_LEFT_BUTTON: {
        int modifiers = glutGetModifiers();
        if (modifiers == GLUT_ACTIVE_SHIFT)
          mouse_mode = PAN;
        else if (modifiers == GLUT_ACTIVE_CTRL)
          mouse_mode = ZOOM;
        else
          mouse_mode = ROTATION;
      } break;
      case GLUT_MIDDLE_BUTTON: // will be removed (?)
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
  glTools::clearBackground((bool)show_background);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glShadeModel(GL_SMOOTH);
  glEnable(GL_DEPTH_TEST);

  if (show_grid == 1) drawGrid();
  if (show_MPs == 1) drawMPs();
  if (show_obstacles == 1) drawObstacles();
  if (show_stress_directions == 1) drawStressDirections();

  if (color_option > 0) colorBar.show(width, height, colorTable);

  textZone.draw();
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
  glColor4f(0.6f, 0.6f, 0.6f, 1.0f);
  glLineWidth(1.0f);

  std::vector<node>& nodes = Conf.nodes;
  size_t* I;
  for (size_t e = 0; e < Conf.Elem.size(); e++) {
    I = &(Conf.Elem[e].I[0]);
    glBegin(GL_LINE_LOOP);
    for (size_t r = 0; r < 4; r++) glVertex2d(nodes[I[r]].pos.x, nodes[I[r]].pos.y);
    glEnd();
  }

  if (show_node_dofs) {
    double s = 0.5 * 0.5 * (Conf.Grid.lx + Conf.Grid.ly);
    glColor4f(1.f, 0.f, 0.0f, 1.0f);
    for (size_t n = 0; n < nodes.size(); n++) {
      if (nodes[n].xfixed) {
        glBegin(GL_POLYGON);
        glVertex2d(nodes[n].pos.x, nodes[n].pos.y);
        glVertex2d(nodes[n].pos.x - s, nodes[n].pos.y + s / 3);
        glVertex2d(nodes[n].pos.x - s, nodes[n].pos.y - s / 3);
        glEnd();
      }
      if (nodes[n].yfixed) {
        glBegin(GL_POLYGON);
        glVertex2d(nodes[n].pos.x, nodes[n].pos.y);
        glVertex2d(nodes[n].pos.x - s / 3, nodes[n].pos.y - s);
        glVertex2d(nodes[n].pos.x + s / 3, nodes[n].pos.y - s);
        glEnd();
      }
    }
  }
}

void precomputeColors() {
  precompColors.clear();
  precompColors.resize(SmoothedData.size());

  switch (color_option) {

    case 0: {
      for (size_t i = 0; i < SmoothedData.size(); i++) {
        precompColors[i].set(204, 204, 230, 255);
      }
    } break;

    case 1: {
      for (size_t i = 0; i < SmoothedData.size(); i++) {
        float vel = (float)norm(SmoothedData[i].vel);
        colorTable.getRGB(vel, &precompColors[i]);
      }
    } break;

    case 2: {
      for (size_t i = 0; i < SmoothedData.size(); i++) {
        float p = 0.5f * (float)(SmoothedData[i].stress.xx + SmoothedData[i].stress.yy);
        colorTable.getRGB(p, &precompColors[i]);
      }
    } break;

    case 3: {
      for (size_t i = 0; i < SmoothedData.size(); i++) {
        float rho = (float)SmoothedData[i].rho;
        colorTable.getRGB(rho, &precompColors[i]);
      }
    } break;

    case 4: {
      for (size_t i = 0; i < SmoothedData.size(); i++) {
        float p = (float)SmoothedData[i].stress.yy;
        colorTable.getRGB(p, &precompColors[i]);
      }
    } break;

    case 5: {
      for (size_t i = 0; i < ADs.size(); i++) {
        if (ADsREF[i].NB == 0.0) continue;
        float D = 1.0f - (float)ADs[i].NB / (float)ADsREF[i].NB;
        colorTable.getRGB(D, &precompColors[i]);
      }
    } break;

    case 6: {
      for (size_t i = 0; i < SmoothedData.size(); i++) {
        float d1 = (float)(SmoothedData[i].velGrad.xx - SmoothedData[i].velGrad.yy);
        float d2 = (float)(SmoothedData[i].velGrad.xy + SmoothedData[i].velGrad.yx);
        float depsq = (float)sqrt(d1 * d1 + d2 * d2);
        colorTable.getRGB(depsq, &precompColors[i]);
      }
    } break;

    case 7: {
      for (size_t i = 0; i < Conf.MP.size(); i++) {
        if (fabs(MPREF[i].F.xx * MPREF[i].F.yy - MPREF[i].F.xy * MPREF[i].F.yx) == 0) continue;
        float volvar = (float)(fabs(Conf.MP[i].F.xx * Conf.MP[i].F.yy - Conf.MP[i].F.xy * Conf.MP[i].F.yx) /
                               fabs(MPREF[i].F.xx * MPREF[i].F.yy - MPREF[i].F.xy * MPREF[i].F.yx)) -
                       1.0f;
        colorTable.getRGB(volvar, &precompColors[i]);
      }
    } break;

    case 8: {
      for (size_t i = 0; i < Conf.MP.size(); i++) {
        colorTable.getRGB((float)Conf.MP[i].contactf.x, &precompColors[i]);
      }
    } break;
    case 9: {
      for (size_t i = 0; i < SmoothedData.size(); i++) {
        float d1 = (float)(SmoothedData[i].velGrad.xx - SmoothedData[i].velGrad.yy);
        float d2 = (float)(SmoothedData[i].velGrad.xy + SmoothedData[i].velGrad.yx);
        float pres = 0.5f * (float)(SmoothedData[i].stress.xx + SmoothedData[i].stress.yy);
        float I = (float)((ADs[i].Rmean / sqrt(d1 * d1 + d2 * d2)) / sqrt((abs(pres) + 1e-6) * SmoothedData[i].rho));
        colorTable.getRGB(I, &precompColors[i]);
      }
    } break;

    default: {
      for (size_t i = 0; i < SmoothedData.size(); i++) {
        precompColors[i].set(204, 204, 230, 255);
      }
    } break;
  }
}

void setColor(int i, float alpha) {
  glColor4f((GLfloat)precompColors[i].r / 255.0f, (GLfloat)precompColors[i].g / 255.0f,
            (GLfloat)precompColors[i].b / 255.0f, alpha);
}

void drawStressDirections() {

  glColor4f(0.0f, 0.0f, 0.0f, 1.0f);

  double fac = 2.5e-6;

  mat4r S, V, D;
  for (size_t i = 0; i < Conf.MP.size(); ++i) {
    double xc = Conf.MP[i].pos.x;
    double yc = Conf.MP[i].pos.y;

    S = SmoothedData[i].stress;
    S.xy = 0.5 * (S.xy + S.yx);
    S.yx = S.xy;
    S.sym_eigen(V, D);

    glBegin(GL_LINES);
    if (fabs(D.xx) > fabs(D.yy)) {
      glLineWidth(2.0f);
      glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
      glVertex2d(xc - fac * V.xx * fabs(D.xx), yc - fac * V.yx * fabs(D.xx));
      glVertex2d(xc + fac * V.xx * fabs(D.xx), yc + fac * V.yx * fabs(D.xx));
      glLineWidth(1.0f);
      glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
      glVertex2d(xc - fac * V.xy * fabs(D.yy), yc - fac * V.yy * fabs(D.yy));
      glVertex2d(xc + fac * V.xy * fabs(D.yy), yc + fac * V.yy * fabs(D.yy));
    } else {
      glLineWidth(1.0f);
      glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
      glVertex2d(xc - fac * V.xx * fabs(D.xx), yc - fac * V.yx * fabs(D.xx));
      glVertex2d(xc + fac * V.xx * fabs(D.xx), yc + fac * V.yx * fabs(D.xx));
      glLineWidth(2.0f);
      glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
      glVertex2d(xc - fac * V.xy * fabs(D.yy), yc - fac * V.yy * fabs(D.yy));
      glVertex2d(xc + fac * V.xy * fabs(D.yy), yc + fac * V.yy * fabs(D.yy));
    }

    glEnd();
  }
}

void drawMPs() {
  glLineWidth(1.0f);

  for (size_t i = 0; i < Conf.MP.size(); ++i) {

    double xc = Conf.MP[i].pos.x;
    double yc = Conf.MP[i].pos.y;
    double R = 0.5 * Conf.MP[i].size;

    if (MP_deformed_shape == 1) {
      setColor((int)i);

      glBegin(GL_POLYGON);
      for (size_t r = 0; r < 4; r++) {
        glVertex2d(SmoothedData[i].corner[r].x, SmoothedData[i].corner[r].y);
      }
      glEnd();

      if (MP_contour == 1) {
        glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
        glBegin(GL_LINE_LOOP);
        for (size_t r = 0; r < 4; r++) {
          glVertex2d(SmoothedData[i].corner[r].x, SmoothedData[i].corner[r].y);
        }
        glEnd();
      }

    } else {
      setColor((int)i);

      glBegin(GL_POLYGON);
      for (double angle = 0.0; angle < 2.0 * M_PI; angle += 0.05 * M_PI) {
        glVertex2d(xc + R * cos(angle), yc + R * sin(angle));
      }
      glEnd();

      if (MP_contour == 1) {
        glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
        glBegin(GL_LINE_LOOP);
        for (double angle = 0.0; angle < 2.0 * M_PI; angle += 0.05 * M_PI) {
          glVertex2d(xc + R * cos(angle), yc + R * sin(angle));
        }
        glEnd();
      }
    }
  }
}

void drawObstacles() {
  for (size_t iObst = 0; iObst < Conf.Obstacles.size(); iObst++) {
    if (Conf.Obstacles[iObst]->getRegistrationName() == "Circle") {
      Circle* C = static_cast<Circle*>(Conf.Obstacles[iObst]);

      glColor4f(0.5f, 0.0f, 0.0f, 0.1f);
      glLineWidth(1.0f);
      glBegin(GL_POLYGON);
      for (double angle = 0.0; angle < 2.0 * M_PI; angle += 0.05 * M_PI) {
        glVertex2d(C->pos.x + C->R * cos(angle), C->pos.y + C->R * sin(angle));
      }
      glEnd();

      glColor4f(0.5f, 0.0f, 0.0f, 1.0f);
      glLineWidth(2.0f);
      glBegin(GL_LINE_LOOP);
      for (double angle = 0.0; angle < 2.0 * M_PI; angle += 0.05 * M_PI) {
        glVertex2d(C->pos.x + C->R * cos(angle), C->pos.y + C->R * sin(angle));
      }
      glEnd();

    } else if (Conf.Obstacles[iObst]->getRegistrationName() == "Line") {
      Line* L = static_cast<Line*>(Conf.Obstacles[iObst]);

      glColor4f(0.5f, 0.0f, 0.0f, 0.1f);
      glLineWidth(1.0f);
      double w = (Conf.Grid.lx + Conf.Grid.ly) * 0.5;
      glBegin(GL_POLYGON);
      glVertex2d(L->pos.x, L->pos.y);
      glVertex2d(L->pos.x + L->len * L->t.x, L->pos.y + L->len * L->t.y);
      glVertex2d(L->pos.x + L->len * L->t.x - w * L->n.x, L->pos.y + L->len * L->t.y - w * L->n.y);
      glVertex2d(L->pos.x - w * L->n.x, L->pos.y - w * L->n.y);
      glEnd();

      glColor4f(0.5f, 0.0f, 0.0f, 1.0f);
      glLineWidth(2.0f);
      glBegin(GL_LINES);
      glVertex2d(L->pos.x, L->pos.y);
      glVertex2d(L->pos.x + L->len * L->t.x, L->pos.y + L->len * L->t.y);
      glEnd();
    }
  }
}

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

bool try_to_readConf(int num, MPMbox& CF, int& OKNum) {
  char file_name[256];
  snprintf(file_name, 256, "conf%d.txt", num);
  if (fileExists(file_name)) {
    OKNum = num;
    char co_file_name[256];
    snprintf(co_file_name, 256, "conf%d.txt_micro", num);
    readConf(file_name, co_file_name, CF);
  } else {
    std::cout << file_name << " does not exist" << std::endl;
    return false;
  }
  return true;
}

void readConf(const char* file_name, const char* co_file_name, MPMbox& CF) {
  std::cout << "Read " << file_name << std::endl;
  CF.clean();
  CF.read(file_name);
  CF.postProcess(SmoothedData);
  precomputeColors();
  textZone.addLine("%s - time = %.3G", file_name, CF.t);
  if (fileExists(co_file_name)) {
    std::cout << "  with additional data in file " << co_file_name << std::endl;
    readAdditionalData(co_file_name);
  }
  if (MPREF.empty()) {
    MPREF = CF.MP;
  }
}

void readAdditionalData(const char* fileName) {
  std::ifstream is(fileName);
  std::string lineTop;
  getline(is, lineTop);
  ADs.clear();
  additionalData D;
  while (is.good()) {
    is >> D.MP_x >> D.MP_y >> D.NInt >> D.NB >> D.TF >> D.FF >> D.Rmean >> D.Vmean >> D.VelMean >> D.VelMin >>
        D.VelMax >> D.VelVar >> D.Vsolid >> D.Vcell >> D.h_xx >> D.h_xy >> D.h_yx >> D.h_yy >> D.ReducedPartDistMean;
    ADs.push_back(D);
  }

  if (ADsREF.empty()) {
    std::cout << "REF ADDITIONAL DATA\n";
    for (size_t i = 0; i < ADs.size(); i++) {
      ADsREF.push_back(ADs[i]);
    }
  }
}

int screenshot(const char* filename) {
  // http://forum.devmaster.net/t/rendering-a-single-frame-to-a-file-with-opengl/12469/2

  // we will store the image data here
  unsigned char* pixels;
  // the thingy we use to write files
  FILE* shot;
  // we get the width/height of the screen into this array
  int screenStats[4];

  // get the width/height of the window
  glGetIntegerv(GL_VIEWPORT, screenStats);

  // generate an array large enough to hold the pixel data
  // (width*height*bytesPerPixel)
  pixels = new unsigned char[screenStats[2] * screenStats[3] * 3];
  // read in the pixel data, TGA's pixels are BGR aligned
  glReadPixels(0, 0, screenStats[2], screenStats[3], GL_BGR, GL_UNSIGNED_BYTE, pixels);

  // open the file for writing. If unsucessful, return 1
  if ((shot = fopen(filename, "wb")) == NULL) return 1;

  // this is the tga header it must be in the beginning of
  // every (uncompressed) .tga
  unsigned char TGAheader[12] = {0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  // the header that is used to get the dimensions of the .tga
  // header[1]*256+header[0] - width
  // header[3]*256+header[2] - height
  // header[4] - bits per pixel
  // header[5] - ?
  unsigned char header[6] = {((unsigned char)(screenStats[2] % 256)),
                             ((unsigned char)(screenStats[2] / 256)),
                             ((unsigned char)(screenStats[3] % 256)),
                             ((unsigned char)(screenStats[3] / 256)),
                             24,
                             0};

  // write out the TGA header
  fwrite(TGAheader, sizeof(unsigned char), 12, shot);
  // write out the header
  fwrite(header, sizeof(unsigned char), 6, shot);
  // write the pixels
  fwrite(pixels, sizeof(unsigned char), screenStats[2] * screenStats[3] * 3, shot);

  // close the file
  fclose(shot);
  // free the memory
  delete[] pixels;

  // return success
  return 0;
}

// =====================================================================
// Main function
// =====================================================================

int main(int argc, char* argv[]) {
  Conf.computationMode = false;
	
  if (argc == 1) {
    confNum = 0;
    try_to_readConf(confNum, Conf, confNum);
  } else if (argc > 1) {
    confNum = 0;
    readConf(argv[1], "###", Conf);
  }

  // ==== Init GLUT and create window
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA);
  glutInitWindowPosition(50, 50);
  glutInitWindowSize(width, height);
  main_window = glutCreateWindow("CONF VISUALIZER (MPMbox)");

  // ==== Register callbacks
  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);

  mouse_mode = NOTHING;

  glText::init();

  glDisable(GL_CULL_FACE);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_COLOR_MATERIAL);

  glEnable(GL_BLEND);
  glBlendEquation(GL_FUNC_ADD);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // ==== Enter GLUT event processing cycle
  fit_view();
  display();
  glutMainLoop();
  return 0;
}
