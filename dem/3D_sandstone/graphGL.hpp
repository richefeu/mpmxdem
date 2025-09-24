#ifndef GRAPHGL_HPP
#define GRAPHGL_HPP

#include <cmath>

#include "glTools.hpp"

class graphGL {
 public:
  // Window position
  int winOffsetX, winOffsetY;
  int winWidth, winHeight;

  // data ranges
  double rangeOffsetX, rangeOffsetY;
  double rangeWidth, rangeHeight;

  bool withLines{false};
  bool withPoints{true};
  GLfloat pointSize{3.0f};
  GLfloat lineWidth{2.0f};
  GLfloat r{0.0f}, g{0.0f}, b{0.0f}, a{1.0f};

  void setColor(GLfloat R, GLfloat G, GLfloat B, GLfloat A) {
    r = R;
    g = G;
    b = B;
    a = A;
  }

  void setDataRanges(double RangeOffsetX, double RangeOffsetY, double RangeWidth, double RangeHeight) {
    rangeOffsetX = RangeOffsetX;
    rangeOffsetY = RangeOffsetY;
    rangeWidth = RangeWidth;
    rangeHeight = RangeHeight;
  }

  void begin(int WinOffsetX, int WinOffsetY, int WinWidth, int WinHeight, const char* title) {
    winOffsetX = WinOffsetX;
    winOffsetY = WinOffsetY;
    winWidth = WinWidth;
    winHeight = WinHeight;

    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

    switch2D::go((GLdouble)viewport[2], (GLdouble)viewport[3]);

    // Frame
    glColor4f(1.0f, 1.0f, 1.0f, 0.95f);
    glBegin(GL_POLYGON);
    glVertex2i(winOffsetX, winOffsetY);
    glVertex2i(winOffsetX + winWidth, winOffsetY);
    glVertex2i(winOffsetX + winWidth, winOffsetY + winHeight);
    glVertex2i(winOffsetX, winOffsetY + winHeight);
    glEnd();

    glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
    glLineWidth(1.5f);
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
    if (X0tick < rangeOffsetX) {
      X0tick += spaceW;
    }

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
    if (Y0tick < rangeOffsetY) Y0tick += spaceH;

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

    // Grid lines
    glColor4f(0.65f, 0.65f, 0.65f, 0.8f);
    for (double Xtick = X0tick + spaceW; Xtick < rangeOffsetX + rangeWidth; Xtick += spaceW) {
      ex = (double)(winWidth) / rangeWidth;
      xo = (int)round(ex * (Xtick - rangeOffsetX)) + winOffsetX;
      glBegin(GL_LINES);
      glVertex2i(xo, winOffsetY);
      glVertex2i(xo, winOffsetY + winHeight);
      glEnd();
    }
    for (double Ytick = Y0tick + spaceH; Ytick < rangeOffsetY + rangeHeight; Ytick += spaceH) {
      ey = (double)(winHeight) / rangeHeight;
      yo = (int)round(ey * (Ytick - rangeOffsetY)) + winOffsetY;
      glBegin(GL_LINES);
      glVertex2i(winOffsetX, yo);
      glVertex2i(winOffsetX + winWidth, yo);
      glEnd();
    }
  }

  void end() { switch2D::back(); }

  double BestTick(double largest, int mostticks, double minTickSpacing = -1.0) {
    double minimum = largest / mostticks;
    double magnitude = pow(10, floor(log10(minimum) / log10(10)));
    double residual = minimum / magnitude;
    double tick;

    // Use a minimum tick spacing to avoid cluttering the graph with too many ticks
    if (residual > 5) {
      tick = 10 * magnitude;
    } else if (residual > 2) {
      tick = 5 * magnitude;
    } else if (residual > 1) {
      tick = 2 * magnitude;
    } else {
      tick = magnitude;
    }

    // Ensure that the tick interval is not smaller than the minimum tick spacing
    if (minTickSpacing > 0.0 && tick < minTickSpacing) {
      tick = minTickSpacing;
    }

    return tick;
  }

  void drawTitle(const char* label) {
    char textValue[50];
    snprintf(textValue, 50, "%s", label);  // to be sure it is null terminated
    int len = 0;
    for (int i = 0; textValue[i] != '\0'; i++) {
      len++;
    }
    glText::print(winOffsetX + (int)round(0.5 * winWidth - 0.5 * len * 9), winOffsetY + winHeight + 4, textValue);
  }

  void drawTickXValue(int xt, double value) {
    char textValue[50];
    snprintf(textValue, 50, "%.2g", value);
    int len = 0;
    for (int i = 0; textValue[i] != '\0'; i++) {
      len++;
    }
    glText::print(xt - (int)round(0.5 * len * 9), winOffsetY - 13, textValue);
  }

  void drawTickYValue(int yt, double value) {
    char textValue[50];
    snprintf(textValue, 50, "%.2g", value);
    int len = 0;
    for (int i = 0; textValue[i] != '\0'; i++) {
      len++;
    }
    glText::print(winOffsetX - (len * 9) - 4, yt - 4, textValue);
  }

  void fit(std::vector<double>& buf, double& valMin, double& valMax) {
    if (buf.empty()) {
      return;
    }

    valMin = buf[0];
    valMax = buf[0];
    for (size_t i = 1; i < buf.size(); i++) {
      if (buf[i] > valMax) {
        valMax = buf[i];
      }

      if (buf[i] < valMin) {
        valMin = buf[i];
      }
    }
  }

  void plot(const std::vector<double>& xbuf, const std::vector<double>& ybuf) {
    if (xbuf.empty() || ybuf.empty() || xbuf.size() != ybuf.size()) {
      std::cerr << "Error: xbuf and ybuf must be non-empty and have the same size" << std::endl;
      std::cerr << "xbuf.size() = " << xbuf.size() << std::endl;
      std::cerr << "ybuf.size() = " << ybuf.size() << std::endl;
      return;
    }

    double ex = (double)(winWidth) / rangeWidth;
    double ey = (double)(winHeight) / rangeHeight;

    glColor3f(0.0f, 0.0f, 0.0f);

    glEnable(GL_SCISSOR_TEST);
    glScissor(winOffsetX + 1, winOffsetY + 1, winWidth - 2, winHeight - 2);

    if (withPoints == true) {
      glPointSize(pointSize);
      glColor4f(r, g, b, a);
      glBegin(GL_POINTS);
      int xo, yo;
      for (size_t i = 0; i < xbuf.size(); i++) {
        xo = (int)round(ex * (xbuf[i] - rangeOffsetX)) + winOffsetX;
        yo = (int)round(ey * (ybuf[i] - rangeOffsetY)) + winOffsetY;
        glVertex2i(xo, yo);
      }
      glEnd();
    }

    if (withLines == true) {
      glLineWidth(lineWidth);
      glColor4f(r, g, b, a);
      glBegin(GL_LINE_STRIP);
      int xo, yo;
      for (size_t i = 0; i < xbuf.size(); i++) {
        xo = (int)round(ex * (xbuf[i] - rangeOffsetX)) + winOffsetX;
        yo = (int)round(ey * (ybuf[i] - rangeOffsetY)) + winOffsetY;
        glVertex2i(xo, yo);
      }
      glEnd();
    }

    glDisable(GL_SCISSOR_TEST);
  }
};

#endif /* end of include guard: GRAPHGL_HPP */
