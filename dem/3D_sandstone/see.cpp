#include "see.hpp"

/*
void clear_background() {
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if (!show_background) return;

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glBegin(GL_QUADS);
  glColor3ub(102, 102, 255);  // Bottom color
  glVertex2f(-1.0f, -1.0f);
  glVertex2f(1.0f, -1.0f);
  glColor3f(1.0f, 1.0f, 1.0f);  // Top color
  glVertex2f(1.0f, 1.0f);
  glVertex2f(-1.0f, 1.0f);
  glEnd();

  glEnable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
}
*/

void printHelp() {

  // Declare the static vector inside the function
  static const std::vector<std::string> keybindLines = {
      "[+]    load next configuration file",
      "[-]    load previous configuration file",
      "[=]    fit the view",
      "[a]    (TODO) switch ON/OFF the fluctuating velocities",
      "[b]    switch ON/OFF the background color",
      "[c]    switch ON/OFF the periodic cell",
      "[d]    switch ON/OFF bond damage",
      "[e][E] decrease/increase particles' alpha (transparence)",
      "[f]    switch ON/OFF the forces",
      "[g]    open another file",
      "[h]    print this help",
      "[k][K] (TODO) decrease/increase the size of velocities arrows",
      "[p]    switch ON/OFF the particles",
      "[q]    quit",
      "[r][R] decrease/increase ghosts' alpha (transparence)",
      "[s]    switch ON/OFF the slice",
      "[t][T] decrease/increase size of forces when displayed as tubes",
      "[w][W] decrease/increase the width of ghost particles",
      "[x]    switch ON/OFF the slice edition",
      "[0]    plot fn vs ft",
      "[1]    plot |mom| vs |ft|",
      "[2]    plot q vs p",
      "[3]    plot q/p vs time"};

  switch2D::go(width, height);
  glColor4f(1.0f, 1.0f, 1.0f, 0.6f);
  glBegin(GL_QUADS);
  int nbLines = keybindLines.size();
  int by = height - nbLines * 15 - 3;
  glVertex2i(0, height);
  glVertex2i(width, height);
  glVertex2i(width, by);
  glVertex2i(0, by);
  glEnd();
  glColor3i(0, 0, 0);
  int dhline = -15;
  int hline = height;

  for (const auto& line : keybindLines) {
    hline += dhline;
    glText::print(15, hline, line.c_str());
  }

  switch2D::back();
}

void select_displayed_plot(int N) {
  if (show_plot == 0) {
    show_plot = 1;
    plot_displayed = N;
  } else {
    if (plot_displayed == N) {
      show_plot = 0;
    } else {
      plot_displayed = N;
    }
  }
}

void keyboard(unsigned char Key, int /*x*/, int /*y*/) {
  switch (Key) {

    case 'b': {
      show_background = 1 - show_background;
    } break;

    case 'c': {
      show_cell = 1 - show_cell;
    } break;

    case 'd': {
      show_bond_damage = 1 - show_bond_damage;
    } break;

    case 'e': {
      if (alpha_particles > 0.1f) alpha_particles -= 0.05f;
    } break;
    case 'E': {
      if (alpha_particles <= 0.95f) alpha_particles += 0.05f;
    } break;

    case 'f': {
      show_forces = 1 - show_forces;
    } break;

    case 'g': {
      show_ghosts = 1 - show_ghosts;
    } break;

    case 'h': {  // printHelp();
      show_help = 1 - show_help;
    } break;

    case 'm': {
      export_sample();
    } break;

    case 'p': {
      show_particles = 1 - show_particles;
    } break;

    case 'q': {
      exit(0);
    } break;

    case 'r': {
      if (alpha_ghosts > 0.1f) {
        alpha_ghosts -= 0.05f;
      }
    } break;
    case 'R': {
      if (alpha_ghosts <= 0.95f) {
        alpha_ghosts += 0.05f;
      }
    } break;

    case 's': {
      show_slice = 1 - show_slice;
    } break;

    case 't': {
      if (forceTubeFactor > 0.2) forceTubeFactor -= 0.05;
    } break;
    case 'T': {
      if (forceTubeFactor <= 0.9) forceTubeFactor += 0.05;
    } break;

    case 'v': {
      show_velocities = 1 - show_velocities;
    } break;

    case 'w': {
      if (ghost_width > 0.1) {
        ghost_width -= 0.1;
      }
    } break;
    case 'W': {
      if (ghost_width <= 0.9) {
        ghost_width += 0.1;
      }
    } break;

    case 'x': {
      display_mode = 1 - display_mode;
    } break;

    case 'z': {
      msg::info("image saved in 'oneshot.tga'", std::cout);
      screenshot("oneshot.tga");
    } break;

    case 'Z': {
      // be carreful there's no way to stop this loop
      // if the process if too long
      while (try_to_readConf(confNum + 1)) {
        char name[256];
        snprintf(name, 256, "shot%d.tga", confNum);
        display();
        screenshot(name);
      }
      msg::info("series of images saved in 'shot<n>.tga'", std::cout);
    } break;

    case '-': {
      if (confNum > 0) {
        try_to_readConf(confNum - 1);
      }
    } break;

    case '+': {
      try_to_readConf(confNum + 1);
    } break;

    case '=': {
      fit_view();
      adjust_clipping_plans();
    } break;

    case '0': {
      select_displayed_plot(0);
    } break;

    case '1': {
      select_displayed_plot(1);
    } break;

    case '2': {
      select_displayed_plot(2);
    } break;

    case '3': {
      select_displayed_plot(3);
    } break;
  };

  glutPostRedisplay();
}

void mouse(int button, int state, int x, int y) {
  if (state == GLUT_UP) {
    mouse_mode = NOTHING;
    // display();
    glutPostRedisplay();
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

vec3r rotatePoint(vec3r const& p, vec3r const& center_, vec3r const& axis, double theta) {
  double const c = cos(theta), s = sin(theta);
  double const C = 1.0 - c;
  vec3r tmp = p - center_;
  return center + vec3r(tmp[0] * (axis[0] * axis[0] * C + c) + tmp[1] * (axis[0] * axis[1] * C - axis[2] * s) +
                            tmp[2] * (axis[0] * axis[2] * C + axis[1] * s),
                        tmp[0] * (axis[1] * axis[0] * C + axis[2] * s) + tmp[1] * (axis[1] * axis[1] * C + c) +
                            tmp[2] * (axis[1] * axis[2] * C - axis[0] * s),
                        tmp[0] * (axis[2] * axis[0] * C - axis[1] * s) +
                            tmp[1] * (axis[2] * axis[1] * C + axis[0] * s) + tmp[2] * (axis[2] * axis[2] * C + c));
}

// Adapted from NeHe production
// glMultMatrixf(Matrix);
// Note that openGL uses a column-major convention for the matrix storage
void quat2GLMatrix(quat& q, GLfloat* pMatrix) {
  // Make sure the matrix has allocated memory to store the rotation data
  if (!pMatrix) return;

  // First row
  pMatrix[0] = 1.0f - 2.0f * (GLfloat)(q.v.y * q.v.y + q.v.z * q.v.z);
  pMatrix[1] = 2.0f * (GLfloat)(q.v.x * q.v.y + q.v.z * q.s);
  pMatrix[2] = 2.0f * (GLfloat)(q.v.x * q.v.z - q.v.y * q.s);
  pMatrix[3] = 0.0f;

  // Second row
  pMatrix[4] = 2.0f * (GLfloat)(q.v.x * q.v.y - q.v.z * q.s);
  pMatrix[5] = 1.0f - 2.0f * (GLfloat)(q.v.x * q.v.x + q.v.z * q.v.z);
  pMatrix[6] = 2.0f * (GLfloat)(q.v.z * q.v.y + q.v.x * q.s);
  pMatrix[7] = 0.0f;

  // Third row
  pMatrix[8] = 2.0f * (GLfloat)(q.v.x * q.v.z + q.v.y * q.s);
  pMatrix[9] = 2.0f * (GLfloat)(q.v.y * q.v.z - q.v.x * q.s);
  pMatrix[10] = 1.0f - 2.0f * (GLfloat)(q.v.x * q.v.x + q.v.y * q.v.y);
  pMatrix[11] = 0.0f;

  // Fourth row
  pMatrix[12] = 0.0f;
  pMatrix[13] = 0.0f;
  pMatrix[14] = 0.0f;
  pMatrix[15] = 1.0f;

  // Now pMatrix[] is a 4x4 homogeneous · that can be applied to an OpenGL Matrix
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

  // display();
  glutPostRedisplay();
}

void normalize(GLfloat* a) {
  GLfloat d = (GLfloat)sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  a[0] /= d;
  a[1] /= d;
  a[2] /= d;
}

void drawtri(GLfloat* a, GLfloat* b, GLfloat* c, int div, float r) {
  if (div <= 0) {
    glNormal3fv(a);
    glVertex3f(-a[0] * r, -a[1] * r, -a[2] * r);
    glNormal3fv(b);
    glVertex3f(-b[0] * r, -b[1] * r, -b[2] * r);
    glNormal3fv(c);
    glVertex3f(-c[0] * r, -c[1] * r, -c[2] * r);
  } else {
    GLfloat ab[3], ac[3], bc[3];
    for (int i = 0; i < 3; i++) {
      ab[i] = (a[i] + b[i]) / 2;
      ac[i] = (a[i] + c[i]) / 2;
      bc[i] = (b[i] + c[i]) / 2;
    }
    normalize(ab);
    normalize(ac);
    normalize(bc);
    drawtri(a, ab, ac, div - 1, r);
    drawtri(b, bc, ab, div - 1, r);
    drawtri(c, ac, bc, div - 1, r);
    drawtri(ab, bc, ac, div - 1, r);
  }
}

void drawsphere(int ndiv, float radius) {
  glBegin(GL_TRIANGLES);
  for (int i = 0; i < 20; ++i)
    drawtri(vdata[tindices[i][0]], vdata[tindices[i][1]], vdata[tindices[i][2]], ndiv, radius);
  glEnd();
}

void add_ghost_pos(size_t i, double mn, double mx, std::vector<vec3r>& lst) {
  lst.clear();
  vec3r pos = box.Particles[i].pos;
  if (pos.x > mn && pos.x < mx && pos.y > mn && pos.y < mx && pos.z > mn && pos.z < mx) return;

  vec3r ghostPos;

  if (pos.x <= mn) {
    ghostPos.set(pos.x + 1.0, pos.y, pos.z);
    lst.push_back(ghostPos);

    if (pos.y <= mn) {
      ghostPos.set(pos.x + 1.0, pos.y + 1.0, pos.z);
      lst.push_back(ghostPos);
    }
    if (pos.y >= mx) {
      ghostPos.set(pos.x + 1.0, pos.y - 1.0, pos.z);
      lst.push_back(ghostPos);
    }
    if (pos.z <= mn) {
      ghostPos.set(pos.x + 1.0, pos.y, pos.z + 1.0);
      lst.push_back(ghostPos);
    }
    if (pos.z >= mx) {
      ghostPos.set(pos.x + 1.0, pos.y, pos.z - 1.0);
      lst.push_back(ghostPos);
    }

    if (pos.y <= mn && pos.z <= mn) {
      ghostPos.set(pos.x + 1.0, pos.y + 1.0, pos.z + 1.0);
      lst.push_back(ghostPos);
    }
    if (pos.y <= mn && pos.z >= mx) {
      ghostPos.set(pos.x + 1.0, pos.y + 1.0, pos.z - 1.0);
      lst.push_back(ghostPos);
    }
    if (pos.y >= mx && pos.z <= mn) {
      ghostPos.set(pos.x + 1.0, pos.y - 1.0, pos.z + 1.0);
      lst.push_back(ghostPos);
    }
    if (pos.y >= mx && pos.z >= mx) {
      ghostPos.set(pos.x + 1.0, pos.y - 1.0, pos.z - 1.0);
      lst.push_back(ghostPos);
    }
  }

  if (pos.x >= mx) {
    ghostPos.set(pos.x - 1.0, pos.y, pos.z);
    lst.push_back(ghostPos);

    if (pos.y <= mn) {
      ghostPos.set(pos.x - 1.0, pos.y + 1.0, pos.z);
      lst.push_back(ghostPos);
    }
    if (pos.y >= mx) {
      ghostPos.set(pos.x - 1.0, pos.y - 1.0, pos.z);
      lst.push_back(ghostPos);
    }
    if (pos.z <= mn) {
      ghostPos.set(pos.x - 1.0, pos.y, pos.z + 1.0);
      lst.push_back(ghostPos);
    }
    if (pos.z >= mx) {
      ghostPos.set(pos.x - 1.0, pos.y, pos.z - 1.0);
      lst.push_back(ghostPos);
    }

    if (pos.y <= mn && pos.z <= mn) {
      ghostPos.set(pos.x - 1.0, pos.y + 1.0, pos.z + 1.0);
      lst.push_back(ghostPos);
    }
    if (pos.y <= mn && pos.z >= mx) {
      ghostPos.set(pos.x - 1.0, pos.y + 1.0, pos.z - 1.0);
      lst.push_back(ghostPos);
    }
    if (pos.y >= mx && pos.z <= mn) {
      ghostPos.set(pos.x - 1.0, pos.y - 1.0, pos.z + 1.0);
      lst.push_back(ghostPos);
    }
    if (pos.y >= mx && pos.z >= mx) {
      ghostPos.set(pos.x - 1.0, pos.y - 1.0, pos.z - 1.0);
      lst.push_back(ghostPos);
    }
  }

  if (pos.y <= mn) {
    ghostPos.set(pos.x, pos.y + 1.0, pos.z);
    lst.push_back(ghostPos);

    if (pos.x <= mn) {
      ghostPos.set(pos.x + 1.0, pos.y + 1.0, pos.z);
      lst.push_back(ghostPos);
    }
    if (pos.x >= mx) {
      ghostPos.set(pos.x - 1.0, pos.y + 1.0, pos.z);
      lst.push_back(ghostPos);
    }
    if (pos.z <= mn) {
      ghostPos.set(pos.x, pos.y + 1.0, pos.z + 1.0);
      lst.push_back(ghostPos);
    }
    if (pos.z >= mx) {
      ghostPos.set(pos.x, pos.y + 1.0, pos.z - 1.0);
      lst.push_back(ghostPos);
    }
  }

  if (pos.y >= mx) {
    ghostPos.set(pos.x, pos.y - 1.0, pos.z);
    lst.push_back(ghostPos);

    if (pos.x <= mn) {
      ghostPos.set(pos.x + 1.0, pos.y - 1.0, pos.z);
      lst.push_back(ghostPos);
    }
    if (pos.x >= mx) {
      ghostPos.set(pos.x - 1.0, pos.y - 1.0, pos.z);
      lst.push_back(ghostPos);
    }
    if (pos.z <= mn) {
      ghostPos.set(pos.x, pos.y - 1.0, pos.z + 1.0);
      lst.push_back(ghostPos);
    }
    if (pos.z >= mx) {
      ghostPos.set(pos.x, pos.y - 1.0, pos.z - 1.0);
      lst.push_back(ghostPos);
    }
  }

  if (pos.z <= mn) {
    ghostPos.set(pos.x, pos.y, pos.z + 1.0);
    lst.push_back(ghostPos);
  }
  if (pos.z >= mx) {
    ghostPos.set(pos.x, pos.y, pos.z - 1.0);
    lst.push_back(ghostPos);
  }
}

// Return true if the real position is inside the slice defined by slice_pos, slice_width and slice_norm
bool inSlice(vec3r& pos) {
  if (show_slice == 0) return true;

  double dst = (pos - slice_pos) * slice_norm;
  double dstMax = 0.5 * slice_width;
  if (dst > dstMax || dst < -dstMax) return false;
  return true;
}

void display() {
  glTools::clearBackground(static_cast<bool>(show_background));
  adjust_clipping_plans();
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  gluLookAt(eye.x, eye.y, eye.z, center.x, center.y, center.z, up.x, up.y, up.z);

  glShadeModel(GL_SMOOTH);
  glEnable(GL_DEPTH_TEST);

  if (show_cell) {
    drawPeriodicCell();
  }
  if (show_velocities) {
    drawVelocities();
  }
  if (show_forces) {
    drawForces();
  }
  if (show_bond_damage) {
    drawBondDamage();
  }
  if (show_particles) {
    drawParticles();
  }
  if (show_ghosts) {
    drawGhosts();
  }

  // mode for slice edition
  if (display_mode == 1) {
    drawSlice();
  }

  if (show_plot) {
    switch (plot_displayed) {
      case 0: {
        plot_fnft();
      } break;
      case 1: {
        plot_ftmom();
      } break;
      case 2: {
        plot_q_vs_p();
      } break;
      case 3: {
        plot_qp();
      } break;
    }
  }

  if (show_help) {
    printHelp();
  }

  textZone.draw();
  glFlush();
  glutSwapBuffers();
}

void plot_fnft() {
  std::vector<double> fn_cont, ft_cont, fn_bond, ft_bond;
  double fnMin = 1e20;
  double fnMax = -1e20;
  double ftMin = 1e20;
  double ftMax = -1e20;
  double Fn, Ft;
  for (size_t k = 0; k < box.Interactions.size(); k++) {
    Fn = box.Interactions[k].fn;
    if (fnMin > Fn) {
      fnMin = Fn;
    }
    if (fnMax < Fn) {
      fnMax = Fn;
    }

    Ft = norm(box.Interactions[k].ft);
    if (ftMin > Ft) {
      ftMin = Ft;
    }
    if (ftMax < Ft) {
      ftMax = Ft;
    }

    if (box.Interactions[k].state >= bondedState) {
      fn_bond.push_back(Fn);
      ft_bond.push_back(Ft);
    } else {
      fn_cont.push_back(Fn);
      ft_cont.push_back(Ft);
    }
  }

  std::vector<double> xCoulomb = {fnMin, fnMax};
  std::vector<double> yCoulomb = {fnMin * box.mu, fnMax * box.mu};

  graphGL Gph;
  Gph.setDataRanges(fnMin, ftMin, fnMax - fnMin, ftMax - ftMin);

  Gph.begin(width / 4, width / 4, width / 2, height / 2, "fn vs |ft|");
  Gph.withPoints = true;
  Gph.withLines = false;
  Gph.pointSize = 4.0f;

  if (!fn_bond.empty()) {
    Gph.setColor(1.0f, 0.0f, 0.0f, 0.5f);
    Gph.plot(fn_bond, ft_bond);
  }

  if (!fn_cont.empty()) {
    Gph.setColor(0.0f, 0.0f, 1.0f, 1.0f);
    Gph.plot(fn_cont, ft_cont);
  }

  Gph.withPoints = false;
  Gph.withLines = true;
  Gph.setColor(0.0f, 0.0f, 0.0f, 1.0f);
  Gph.plot(xCoulomb, yCoulomb);
  Gph.end();
}

void plot_ftmom() {
  std::vector<double> mom_bond, ft_bond;
  // double momMin = 1e20;
  double momMax = -1e20;
  // double ftMin = 1e20;
  double ftMax = -1e20;
  double Mom, Ft;
  for (size_t k = 0; k < box.Interactions.size(); k++) {
    if (box.Interactions[k].state < bondedState) {
      continue;
    }

    Mom = norm(box.Interactions[k].mom);
    // if (momMin > Mom) momMin = Mom;
    if (momMax < Mom) {
      momMax = Mom;
    }

    Ft = norm(box.Interactions[k].ft);
    // if (ftMin > Ft) ftMin = Ft;
    if (ftMax < Ft) {
      ftMax = Ft;
    }

    mom_bond.push_back(Mom);
    ft_bond.push_back(Ft);
  }

  graphGL Gph;
  Gph.setDataRanges(0.0, 0.0, momMax, ftMax);

  Gph.begin(width / 4, width / 4, width / 2, height / 2, "|mom| vs |ft|");
  Gph.withPoints = true;
  Gph.withLines = false;
  Gph.pointSize = 4.0f;
  if (!mom_bond.empty()) {
    Gph.setColor(1.0f, 0.0f, 0.0f, 0.5f);
    Gph.plot(mom_bond, ft_bond);
  }

  Gph.end();
}

void plot_qp() {
  std::vector<double> Time, qp;
  std::vector<mat9r> Str;
  if (fileExists("stress.out.txt")) {
    std::ifstream file("stress.out.txt");
    double t;
    mat9r S;
    while (file) {
      file >> t >> S;
      Time.push_back(t);
      Str.push_back(S);
      qp.push_back(0.0);
    }
  } else {
    msg::alert("File 'stress.out.txt' not found", std::cout);
    return;
  }

  // curve
  double tMin = 1e20;
  double tMax = -1e20;
  double qpMin = 1e20;
  double qpMax = -1e20;
  mat9r V;
  vec3r D;
  vec3r U;
  U.set(1.0 / sqrt(3.0));
  double q, p, QP;
  for (size_t i = 0; i < Time.size(); i++) {
    if (tMin > Time[i]) {
      tMin = Time[i];
    }
    if (tMax < Time[i]) {
      tMax = Time[i];
    }

    Str[i].sym_eigen(V, D);
    q = norm(D - (D * U) * U);
    p = 0.333333333 * (D.x + D.y + D.z);
    if (p != 0.0) {
      QP = q / p;
    } else {
      QP = 0.0;
    }
    if (qpMin > QP) {
      qpMin = QP;
    }
    if (qpMax < QP) {
      qpMax = QP;
    }
    qp[i] = QP;
  }

  // point
  size_t itime = 0;
  double dtMin = fabs(Time[0] - box.t);
  for (size_t i = 0; i < Time.size(); i++) {
    double dt = fabs(Time[i] - box.t);
    if (dt < dtMin) {
      dtMin = dt;
      itime = i;
    }
  }
  std::vector<double> xp = {Time[itime]};
  std::vector<double> yp = {qp[itime]};

  graphGL Gph;
  Gph.setDataRanges(tMin, qpMin, tMax - tMin, qpMax - qpMin);

  Gph.begin(70, width / 4, width - 90, height / 2, "q/p vs. time");
  Gph.withPoints = false;
  Gph.withLines = true;
  Gph.setColor(0.0f, 0.0f, 1.0f, 1.0f);
  Gph.plot(Time, qp);
  Gph.withPoints = true;
  Gph.withLines = false;
  Gph.setColor(1.0f, 0.0f, 0.0f, 0.6f);
  Gph.pointSize = 15.0f;
  Gph.plot(xp, yp);
  Gph.end();
}

void plot_q_vs_p() {
  std::vector<double> qvec, pvec;
  std::vector<mat9r> Str;
  size_t itime = 0;

  if (fileExists("stress.out.txt")) {
    std::ifstream file("stress.out.txt");
    double t;
    mat9r S;

    double dtMin = 1e20;
    size_t i = 0;
    while (file) {
      file >> t >> S;
      double dt = fabs(t - box.t);
      if (dt < dtMin) {
        dtMin = dt;
        itime = i;
      }
      Str.push_back(S);
      qvec.push_back(0.0);
      pvec.push_back(0.0);
      i++;
    }
  } else {
    msg::alert("File 'stress.out.txt' not found", std::cout);
    return;
  }

  double qMin = 1e20;
  double qMax = -1e20;
  double pMin = 1e20;
  double pMax = -1e20;
  mat9r V;
  vec3r D;
  vec3r U;
  U.set(1.0 / sqrt(3.0));
  double q, p;
  for (size_t i = 0; i < pvec.size(); i++) {

    Str[i].sym_eigen(V, D);
    q = norm(D - (D * U) * U);
    p = 0.333333333 * (D.x + D.y + D.z);

    if (qMin > q) {
      qMin = q;
    }
    if (qMax < q) {
      qMax = q;
    }
    if (pMin > p) {
      pMin = p;
    }
    if (pMax < p) {
      pMax = p;
    }

    pvec[i] = p;
    qvec[i] = q;
  }

  // point
  std::vector<double> xp = {pvec[itime]};
  std::vector<double> yp = {qvec[itime]};

  graphGL Gph;
  Gph.setDataRanges(pMin, qMin, pMax - pMin, qMax - qMin);

  Gph.begin(width / 4, height / 4, width / 2, height / 2, "q vs p");
  Gph.withPoints = false;
  Gph.withLines = true;
  Gph.plot(pvec, qvec);
  Gph.withPoints = true;
  Gph.withLines = false;
  Gph.setColor(1.0f, 0.0f, 0.0f, 0.6f);
  Gph.pointSize = 15.0f;
  Gph.plot(xp, yp);
  Gph.end();
}

void adjust_clipping_plans() {
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  wh_ratio = (float)width / (float)height;
  float zf = (float)((eye - center).normalize());
  vec3r mx = component_max(box.Cell.h.get_xcol(), box.Cell.h.get_ycol());
  mx = component_max(mx, box.Cell.h.get_zcol());
  max_length = (GLfloat)norm(mx);
  znear = zf - 0.5f * max_length;
  float close_dst = 0.1f * zf;
  if (znear < close_dst) znear = close_dst;
  zfar = zf + 0.5f * max_length;
  gluPerspective(view_angle, wh_ratio, znear, zfar);
  glMatrixMode(GL_MODELVIEW);
}

void fit_view() {
  vec3r dir = (eye - center);
  vec3r diag = box.Cell.h.get_xcol() + box.Cell.h.get_ycol() + box.Cell.h.get_zcol();
  dir.normalize();
  center = 0.5 * diag;
  GLfloat d = 0.5f * (GLfloat)diag.length() / (GLfloat)(atan(view_angle * M_PI / 360.0));
  eye = center + d * dir;
}

void reshape(int w, int h) {
  width = w;
  height = h;
  glViewport(0, 0, width, height);

  adjust_clipping_plans();
  glutPostRedisplay();
}

// This function draw the periodic cell
// ====================================
//      Z
//      4______7
//     /|     /|
//   5/_|___6/ |
//   |  0___|__3 Y
//   | /    | /
//   1/_____2/
//   X
// ====================================
void drawPeriodicCell() {
  glDisable(GL_LIGHTING);

  glLineWidth(3.0f);
  // glColor3f(0.8f, 0.2f, 0.2f);
  glColor3f(1.0f, 0.25f, 0.1f);

  vec3r p0;
  vec3r p1 = box.Cell.h.get_xcol();
  vec3r p3 = box.Cell.h.get_ycol();
  vec3r p2 = p1 + p3;
  vec3r p4 = p0 + box.Cell.h.get_zcol();
  vec3r p5 = p1 + box.Cell.h.get_zcol();
  vec3r p6 = p2 + box.Cell.h.get_zcol();
  vec3r p7 = p3 + box.Cell.h.get_zcol();

  glBegin(GL_LINE_LOOP);
  glVertex3d(p0.x, p0.y, p0.z);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glVertex3d(p0.x, p0.y, p0.z);
  glEnd();

  glBegin(GL_LINE_LOOP);
  glVertex3d(p4.x, p4.y, p4.z);
  glVertex3d(p5.x, p5.y, p5.z);
  glVertex3d(p6.x, p6.y, p6.z);
  glVertex3d(p7.x, p7.y, p7.z);
  glVertex3d(p4.x, p4.y, p4.z);
  glEnd();

  glBegin(GL_LINES);
  glVertex3d(p0.x, p0.y, p0.z);
  glVertex3d(p4.x, p4.y, p4.z);

  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p5.x, p5.y, p5.z);

  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p6.x, p6.y, p6.z);

  glVertex3d(p3.x, p3.y, p3.z);
  glVertex3d(p7.x, p7.y, p7.z);
  glEnd();
}

void drawSlice()  // TO BE CONTINUED
{
  vec3r orig = slice_pos - 0.5 * slice_width * slice_norm;
  vec3r dest = slice_pos + 0.5 * slice_width * slice_norm;

  vec3r v = slice_norm;
  vec3r vmz(v.x, v.y, v.z - 1.0);  // v - z

  vec3r a;
  if (norm2(vmz) > 0.01)
    a.set(v.y, -v.x, 0.0);
  else
    a.set(-v.z, 0.0, v.x);
  vec3r b = cross(v, a);

  vec3r c;
  double r = 0.5 * max_length;

  glDisable(GL_LIGHTING);
  glColor4f(0.3f, 1.0f, 0.3f, 0.25f);

  glBegin(GL_TRIANGLE_FAN);
  glVertex3d(orig.x, orig.y, orig.z);
  for (double angle = 0.0; angle <= 2.0 * M_PI; angle += 0.2 * M_PI) {
    c = cos(angle) * a + sin(angle) * b;
    glNormal3d(c.x, c.y, c.z);
    c = orig + r * c;
    glVertex3d(c.x, c.y, c.z);
  }
  glEnd();

  glBegin(GL_TRIANGLE_FAN);
  glVertex3d(dest.x, dest.y, dest.z);
  for (double angle = 0.0; angle <= 2.0 * M_PI; angle += 0.2 * M_PI) {
    c = cos(angle) * a + sin(angle) * b;
    glNormal3d(c.x, c.y, c.z);
    c = dest + r * c;
    glVertex3d(c.x, c.y, c.z);
  }
  glEnd();
}

void drawArrow(vec3r& orig, vec3r& arrow) {
  vec3r dest = orig + arrow;

  glLineWidth(2.0f);
  glBegin(GL_LINES);
  glVertex3d(orig.x, orig.y, orig.z);
  glVertex3d(dest.x, dest.y, dest.z);
  glEnd();

  vec3r v = arrow;
  v.normalize();
  vec3r vmz(v.x, v.y, v.z - 1.0);  // v - z

  vec3r a;
  if (norm2(vmz) > 0.1)
    a.set(v.y, -v.x, 0.0);
  else
    a.set(-v.z, 0.0, v.x);
  a.normalize();
  vec3r b = cross(v, a);

  vec3r head = dest - arrowSize * v;
  vec3r c;
  double r = arrowSize * tan(0.5 * arrowAngle);
  glBegin(GL_TRIANGLE_FAN);
  glVertex3d(dest.x, dest.y, dest.z);
  for (double angle = 0.0; angle <= 2.0 * M_PI; angle += 0.2 * M_PI) {
    c = cos(angle) * a + sin(angle) * b;
    glNormal3d(c.x, c.y, c.z);  // Pas tout à fait juste (!) Mais c'est pas grave, c'est pour le calcul de l'ombre
    c = head + r * c;
    glVertex3d(c.x, c.y, c.z);
  }
  glEnd();
}

void drawTube(vec3r& orig, vec3r& arrow, double diam) {
  vec3r dest = orig + arrow;
  vec3r v = arrow;
  v.normalize();
  vec3r vmz(v.x, v.y, v.z - 1.0);  // v - z

  vec3r a;
  if (norm2(vmz) > 0.1)
    a.set(v.y, -v.x, 0.0);
  else
    a.set(-v.z, 0.0, v.x);
  a.normalize();
  vec3r b = cross(v, a);

  vec3r c1, c2, n;
  double r = 0.5 * diam;
  glBegin(GL_TRIANGLE_STRIP);
  for (double angle = 0.0; angle <= 2.0 * M_PI; angle += 0.1 * M_PI) {
    n = cos(angle) * a + sin(angle) * b;
    glNormal3d(n.x, n.y, n.z);
    n *= r;
    c1 = orig + n;
    c2 = dest + n;
    glVertex3d(c1.x, c1.y, c1.z);
    glVertex3d(c2.x, c2.y, c2.z);
  }
  glEnd();
}

void drawParticles() {
  if (mouse_mode != NOTHING && box.Particles.size() > 100) return;

  GLColorRGBA color;
  glEnable(GL_LIGHTING);
  for (size_t i = 0; i < box.Particles.size(); ++i) {
    color = colorParticle((int)i);
    glColor4f(color.r, color.g, color.b, color.a);
    vec3r pos = box.Cell.h * box.Particles[i].pos;
    if (!inSlice(pos)) continue;
    glPushMatrix();
    glTranslated(pos.x, pos.y, pos.z);
    // Rotation is not necessary since the particles are spheres
    // quat2GLMatrix (box.Particles[i].Q, Rot_Matrix);
    // glMultMatrixf (Rot_Matrix);
    drawsphere(2, (float)box.Particles[i].radius);
    glPopMatrix();
  }
}

void drawGhosts() {
  if (mouse_mode != NOTHING && box.Particles.size() > 100) return;

  std::vector<vec3r> lst_pos;  // list of reduced positions of ghost particles
  double mn = ghost_width;
  double mx = 1.0 - ghost_width;
  GLColorRGBA color;
  for (size_t i = 0; i < box.Particles.size(); ++i) {
    add_ghost_pos(i, mn, mx, lst_pos);
    for (size_t ig = 0; ig < lst_pos.size(); ig++) {
      color = colorParticle((int)i);
      glColor4f(color.r, color.g, color.b, alpha_ghosts);
      vec3r pos = box.Cell.h * lst_pos[ig];
      if (!inSlice(pos)) continue;
      glPushMatrix();
      glTranslated(pos.x, pos.y, pos.z);
      drawsphere(2, (float)box.Particles[i].radius);
      glPopMatrix();
    }
  }
}

void drawForces() {
  if (mouse_mode != NOTHING && box.Particles.size() > 2000) return;

  // Scaling
  double scal = forceTubeFactor * radiusMax / box.FnMax;

  glEnable(GL_LIGHTING);
  // GLColorRGBA color;
  size_t i, j;
  double diam;
  for (size_t k = 0; k < box.Interactions.size(); k++) {
    i = box.Interactions[k].i;
    j = box.Interactions[k].j;

    vec3r orig = box.Cell.h * box.Particles[i].pos;
    if (!inSlice(orig)) continue;

    diam = box.Interactions[k].fn;

    vec3r sij = box.Particles[j].pos - box.Particles[i].pos;
    vec3r dec(floor(sij.x + 0.5), floor(sij.y + 0.5), floor(sij.z + 0.5));
    sij -= dec;
    vec3r branch = box.Cell.h * sij;

    // color = colorForce(i);
    if (box.Interactions[k].state == bondedState) {
      glColor4f(0.4f, 0.4f, 1.0f, 1.0f);
    } else if (box.Interactions[k].state == bondedStateDam) {
      glColor4f(0.4f, 0.4f, 1.0f, 1.0f);
    } else {
      glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
    }
    drawTube(orig, branch, scal * diam);

    if (fabs(dec.x) > 0.0) {
      vec3r da(box.Cell.h.xx * dec.x, box.Cell.h.yx * dec.x, box.Cell.h.zx * dec.x);
      orig += da;
      drawTube(orig, branch, scal * diam);
    }
    if (fabs(dec.y) > 0.0) {
      vec3r db(box.Cell.h.xy * dec.y, box.Cell.h.yy * dec.y, box.Cell.h.zy * dec.y);
      orig += db;
      drawTube(orig, branch, scal * diam);
    }
    if (fabs(dec.z) > 0.0) {
      vec3r dc(box.Cell.h.xz * dec.x, box.Cell.h.yz * dec.z, box.Cell.h.zz * dec.z);
      orig += dc;
      drawTube(orig, branch, scal * diam);
    }
  }
}

void drawBondDamage() {
  if (mouse_mode != NOTHING && box.Particles.size() > 2000) return;

  // Scaling
  double scal = 0.5 * radiusMean;

  glEnable(GL_LIGHTING);
  size_t i, j;
  double diameter;
  for (size_t k = 0; k < box.Interactions.size(); k++) {
    if (box.Interactions[k].state < bondedState) {
      continue;
    }
    i = box.Interactions[k].i;
    j = box.Interactions[k].j;

    vec3r orig = box.Cell.h * box.Particles[i].pos;
    if (!inSlice(orig)) continue;

    diameter = 1.0 - box.Interactions[k].D;

    vec3r sij = box.Particles[j].pos - box.Particles[i].pos;
    vec3r dec(floor(sij.x + 0.5), floor(sij.y + 0.5), floor(sij.z + 0.5));
    sij -= dec;
    vec3r branch = box.Cell.h * sij;

    // glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
    glColor4f(0.35f, 0.35f, 0.35f, 1.0f);
    drawTube(orig, branch, scal * diameter);
    if (fabs(dec.x) > 0.0) {
      vec3r da(box.Cell.h.xx * dec.x, box.Cell.h.yx * dec.x, box.Cell.h.zx * dec.x);
      orig += da;
      drawTube(orig, branch, scal * diameter);
    }
    if (fabs(dec.y) > 0.0) {
      vec3r db(box.Cell.h.xy * dec.y, box.Cell.h.yy * dec.y, box.Cell.h.zy * dec.y);
      orig += db;
      drawTube(orig, branch, scal * diameter);
    }
    if (fabs(dec.z) > 0.0) {
      vec3r dc(box.Cell.h.xz * dec.x, box.Cell.h.yz * dec.z, box.Cell.h.zz * dec.z);
      orig += dc;
      drawTube(orig, branch, scal * diameter);
    }
  }
}

void drawVelocities() {
  // Scaling
  double velMax = 0.0;
  double velSqr;
  vec3r Vel;
  for (size_t i = 0; i < box.Particles.size(); ++i) {
    Vel = box.Cell.vh * box.Particles[i].pos + box.Cell.h * box.Particles[i].vel;  // affine + fluctuation
    velSqr = norm2(Vel);
    if (velSqr > velMax) velMax = velSqr;
  }
  if (velMax == 0.0) return;
  velMax = sqrt(velMax);
  double scal = 5 * radiusMax / velMax;

  arrowSize = radiusMin / 2.0;

  glEnable(GL_LIGHTING);
  glColor4f(0.2f, 0.2f, 0.2f, 1.0f);
  GLColorRGBA color;
  for (size_t i = 0; i < box.Particles.size(); ++i) {
    vec3r pos = box.Cell.h * box.Particles[i].pos;
    if (!inSlice(pos)) continue;

    Vel = scal * (box.Cell.vh * box.Particles[i].pos + box.Cell.h * box.Particles[i].vel);
    if (1) {
      color = colorParticleVelocityMagnitude((int)i);
      glColor4f(color.r, color.g, color.b, color.a);
    }
    drawArrow(pos, Vel);
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

bool try_to_readConf(int num) {
  char file_name[256];
  snprintf(file_name, 256, "conf%d", num);
  if (fileExists(file_name)) {
    std::cout << "Read " << file_name << std::endl;
    box.clearMemory();
    box.loadConf(file_name);
    confNum = box.iconf;
    textZone.addLine("conf-file: %s (time = %f)", file_name, box.t);
    adjust_clipping_plans();
  } else {
    std::cout << file_name << " does not exist" << std::endl;
    return false;
  }
  return true;
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

void export_sample() {
  std::ofstream file("sample.txt");

  file << box.Cell.h.xx << ' ' << box.Cell.h.xy << ' ' << box.Cell.h.xz << '\n';
  file << box.Cell.h.yx << ' ' << box.Cell.h.yy << ' ' << box.Cell.h.yz << '\n';
  file << box.Cell.h.zx << ' ' << box.Cell.h.zy << ' ' << box.Cell.h.zz << '\n';

  for (size_t i = 0; i < box.Particles.size(); i++) {
    vec3r pos = box.Cell.h * box.Particles[i].pos;
    file << pos << ' ' << box.Particles[i].radius << '\n';
  }

  // std::cout << "Sample has been exported in sample.txt\n";
  msg::info("Sample has been exported in sample.txt", std::cout);
}

void menu(int num) {
  switch (num) {

    case 0:
      exit(0);
      break;

    case 100:
      colorParticle = colorParticleNone;
      break;
    case 101: {
      colorParticle = colorParticleVelocityMagnitude;
      ParticleColorTable.setMinMax(0.0f, (float)box.VelMax);
      ParticleColorTable.Rebuild();
    } break;
  };

  glutPostRedisplay();
}

void buildMenu() {
  int submenu1 = glutCreateMenu(menu);  // Particle Colors
  glutAddMenuEntry("None", 100);
  glutAddMenuEntry("Velocity Magnitude", 101);
  glutAddMenuEntry("Sum of Normal Contact Forces", 102);

  int submenu2 = glutCreateMenu(menu);  // Force Colors
  glutAddMenuEntry("None", 200);
  glutAddMenuEntry("Magnitude", 201);

  glutCreateMenu(menu);  // Main menu
  glutAddSubMenu("Particle Colors", submenu1);
  glutAddSubMenu("Force Colors", submenu2);
  glutAddSubMenu("Velocity Colors", submenu2);
  glutAddMenuEntry("Quit", 0);
}

// =====================================================================
// Colorizer functions
// =====================================================================

GLColorRGBA colorForceNone(int /* i */) { return GLColorRGBA(0.2f, 0.2f, 0.2f, 1.0f); }
GLColorRGBA colorParticleNone(int /* i */) { return GLColorRGBA(0.5f, 0.5f, 0.5f, alpha_particles); }

GLColorRGBA colorParticleVelocityMagnitude(int i) {
  colorRGBA RGBA;
  float vel = (float)norm(box.Cell.vh * box.Particles[i].pos + box.Cell.h * box.Particles[i].vel);
  ParticleColorTable.getRGB(vel, &RGBA);
  return GLColorRGBA(RGBA.rr, RGBA.gg, RGBA.bb, 1.0f);
}

// =====================================================================
// Main function
// =====================================================================

int main(int argc, char* argv[]) {
  INIT_TIMERS();

  char file_name[256];
  if (argc == 1) {
    snprintf(file_name, 256, "conf0");
    box.loadConf(file_name);
    confNum = box.iconf;
  } else {
    snprintf(file_name, 256, "%s", argv[1]);
    box.loadConf(file_name);
    confNum = box.iconf;
  }

  if (box.Particles.empty()) {
    std::cerr << "No particles! Goodbye." << std::endl;
    return 1;
  }

  radiusMean = radiusMin = radiusMax = box.Particles[0].radius;
  double r;
  for (size_t i = 1; i < box.Particles.size(); i++) {
    r = box.Particles[i].radius;
    if (radiusMin > r) radiusMin = r;
    if (radiusMax < r) radiusMax = r;
    radiusMean += r;
  }
  radiusMean /= (double)(box.Particles.size());

  DamCol.setMinMax(0.0, 1.0);
  DamCol.setTableID(18);
  DamCol.Rebuild();

  // ==== Init GLUT and create window
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
  glutInitWindowPosition(50, 50);
  glutInitWindowSize(width, height);
  main_window = glutCreateWindow("PBC3Dbox VISUALIZER");

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

  slice_pos = 0.5 * (box.Cell.h.get_xcol() + box.Cell.h.get_ycol() + box.Cell.h.get_zcol());
  slice_norm.set(1.0, 0.0, 0.0);
  slice_width = 2 * box.Particles[0].radius;

  mouse_mode = NOTHING;
  view_angle = 45.0;
  znear = 0.01f;
  zfar = 10.0f;

  glText::init();
  textZone.addLine("conf-file: %s (time = %f)", file_name, box.t);

  glDisable(GL_CULL_FACE);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHT1);
  glEnable(GL_COLOR_MATERIAL);

  // Create light components
  GLfloat ambientLight[] = {0.2f, 0.2f, 0.2f, 1.0f};
  GLfloat diffuseLight[] = {0.8f, 0.8f, 0.8f, 1.0f};
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
