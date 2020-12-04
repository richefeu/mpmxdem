#ifndef GRAPH_HPP_184DF783
#define GRAPH_HPP_184DF783

#include <cmath>

class graphGL {
public:
  // Window position
  int winOffsetX, winOffsetY;
  int winWidth, winHeight;

  // data ranges
  double rangeOffsetX, rangeOffsetY;
  double rangeWidth, rangeHeight;

  void setDataRanges(double RangeOffsetX, double RangeOffsetY, double RangeWidth, double RangeHeight) {
    rangeOffsetX = RangeOffsetX;
    rangeOffsetY = RangeOffsetY;
    rangeWidth = RangeWidth;
    rangeHeight = RangeHeight;
  }

  void begin(int WinOffsetX, int WinOffsetY, int WinWidth, int WinHeight, const char *title) {
    winOffsetX = WinOffsetX;
    winOffsetY = WinOffsetY;
    winWidth = WinWidth;
    winHeight = WinHeight;

    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();

    glLoadIdentity();
    glOrtho(0.0f, (GLdouble)viewport[2], 0.0f, (GLdouble)viewport[3], -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);

    glColor4f(1.0f, 1.0f, 1.0f, 0.95f);
    glBegin(GL_POLYGON);
    glVertex2i(winOffsetX, winOffsetY);
    glVertex2i(winOffsetX + winWidth, winOffsetY);
    glVertex2i(winOffsetX + winWidth, winOffsetY + winHeight);
    glVertex2i(winOffsetX, winOffsetY + winHeight);
    glEnd();

    glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
    glBegin(GL_LINE_LOOP);
    glVertex2i(winOffsetX, winOffsetY);
    glVertex2i(winOffsetX + winWidth, winOffsetY);
    glVertex2i(winOffsetX + winWidth, winOffsetY + winHeight);
    glVertex2i(winOffsetX, winOffsetY + winHeight);
    glEnd();

    // Title
    drawTitle(title);

    // Tick marks along X
    double spaceW = BestTick(rangeWidth, 4);
    double X0tick = floor(rangeOffsetX / spaceW) * spaceW;
    if (X0tick < rangeOffsetX)
      X0tick += spaceW;

    double ex;
    int xo;
    for (double Xtick = X0tick; Xtick < rangeOffsetX + rangeWidth; Xtick += spaceW) {
      ex = (double)(winWidth) / rangeWidth;
      xo = (int)round(ex * (Xtick - rangeOffsetX)) + winOffsetX;
      glBegin(GL_LINES);
      glVertex2i(xo, winOffsetY);
      glVertex2i(xo, winOffsetY + 6);

      glVertex2i(xo, winOffsetY + winHeight);
      glVertex2i(xo, winOffsetY + winHeight - 6);
      glEnd();

      drawTickXValue(xo, Xtick);
    }

    // Tick marks along Y
    double spaceH = BestTick(rangeHeight, 4);
    double Y0tick = floor(rangeOffsetY / spaceH) * spaceH;
    if (Y0tick < rangeOffsetY)
      Y0tick += spaceH;

    double ey;
    int yo;
    for (double Ytick = Y0tick; Ytick < rangeOffsetY + rangeHeight; Ytick += spaceH) {
      ey = (double)(winHeight) / rangeHeight;
      yo = (int)round(ey * (Ytick - rangeOffsetY)) + winOffsetY;
      glBegin(GL_LINES);
      glVertex2i(winOffsetX, yo);
      glVertex2i(winOffsetX + 6, yo);

      glVertex2i(winOffsetX + winWidth, yo);
      glVertex2i(winOffsetX + winWidth - 6, yo);
      glEnd();

      drawTickYValue(yo, Ytick);
    }
  }

  void end() {
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
  }

  // adapted from http://stackoverflow.com/questions/361681/algorithm-for-nice-grid-line-intervals-on-a-graph
  double BestTick(double largest, int mostticks) {
    double minimum = largest / mostticks;
    double magnitude = pow(10, floor(log10(minimum) / log10(10)));
    double residual = minimum / magnitude;
    double tick;
    if (residual > 5)
      tick = 10 * magnitude;
    else if (residual > 2)
      tick = 5 * magnitude;
    else if (residual > 1)
      tick = 2 * magnitude;
    else
      tick = magnitude;

    return tick;
  }

  void drawTitle(const char *label) {
    char textValue[50];
    sprintf(textValue, "%s", label); // to be sure it is null terminated
    int len = 0;
    for (int i = 0; textValue[i] != '\0'; i++)
      len++;
    glRasterPos2i(winOffsetX + (int)round(0.5 * winWidth - 0.5 * len * 9), winOffsetY + winHeight + 4);
    for (int i = 0; textValue[i] != '\0'; i++)
      glutBitmapCharacter(GLUT_BITMAP_9_BY_15, textValue[i]);
  }

  void drawTickXValue(int xt, double value) {
    char textValue[50];
    sprintf(textValue, "%.2g", value);
    int len = 0;
    for (int i = 0; textValue[i] != '\0'; i++)
      len++;
    glRasterPos2i(xt - (int)round(0.5 * len * 8), winOffsetY - 13);
    for (int i = 0; textValue[i] != '\0'; i++)
      glutBitmapCharacter(GLUT_BITMAP_8_BY_13, textValue[i]);
  }

  void drawTickYValue(int yt, double value) {
    char textValue[50];
    sprintf(textValue, "%.2g", value);
    int len = 0;
    for (int i = 0; textValue[i] != '\0'; i++)
      len++;
    glRasterPos2i(winOffsetX - (len * 8) - 4, yt - 4);
    for (int i = 0; textValue[i] != '\0'; i++)
      glutBitmapCharacter(GLUT_BITMAP_8_BY_13, textValue[i]);
  }

  void fit(std::vector<double> &buf, double &valMin, double &valMax) {
    if (buf.empty())
      return;

    valMin = buf[0];
    valMax = buf[0];
    for (size_t i = 1; i < buf.size(); i++) {
      if (buf[i] > valMax)
        valMax = buf[i];

      if (buf[i] < valMin)
        valMin = buf[i];
    }
  }

  void plot(std::vector<double> &xbuf, std::vector<double> &ybuf, int opt) {
    double ex, ey;
    if (rangeWidth != 0.0)
      ex = (double)(winWidth) / rangeWidth;
    else
      return;
    if (rangeHeight != 0.0)
      ey = (double)(winHeight) / rangeHeight;
    else
      return;

    glColor3f(0.0f, 0.0f, 0.0f);

    glEnable(GL_SCISSOR_TEST);
    glScissor(winOffsetX + 1, winOffsetY + 1, winWidth - 2, winHeight - 2);

    if (opt == 0) {
      glPointSize(2.0f);
      glBegin(GL_POINTS);
    } else {
      glLineWidth(2.0f);
      glBegin(GL_LINE_STRIP);
    }

    int xo, yo;
    for (size_t i = 0; i < xbuf.size(); i++) {
      xo = (int)round(ex * (xbuf[i] - rangeOffsetX)) + winOffsetX;
      yo = (int)round(ey * (ybuf[i] - rangeOffsetY)) + winOffsetY;
      glVertex2i(xo, yo);
    }

    glEnd();

    glDisable(GL_SCISSOR_TEST);
  }
};

#endif /* end of include guard: GRAPH_HPP_184DF783 */
