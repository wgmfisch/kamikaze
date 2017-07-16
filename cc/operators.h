#include <opencv2/opencv.hpp>

cv::Size operator/(cv::Size s, int d) {
  s.width /= d;
  s.height /= d;
  return s;
}

cv::Point operator+(cv::Point p, cv::Size s) {
  p.x += s.width;
  p.y += s.height;
  return p;
}

cv::Point operator-(cv::Point p, int scalar) {
  p.x -= scalar;
  p.y -= scalar;
  return p;
}

#define POINT_OP(OP)                                                           \
  cv::Point operator OP(cv::Point l, cv::Point r) {                            \
    return {l.x OP r.x, l.y OP r.y};                                           \
  }

POINT_OP(/);
POINT_OP(+);
POINT_OP(-);
POINT_OP(*);
#undef POINT_OP

cv::Point Center(cv::Rect r) {
  return r.tl() + r.size() / 2;
}

double sqdist(cv::Point a, cv::Point b) {
  return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}

