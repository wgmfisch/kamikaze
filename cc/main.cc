#include <atomic>
#include <chrono>
#include <condition_variable>
#include <ctime>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include <gflags/gflags.h>

#include "logging.h"
#include "robot.h"

//#define USE_CUDA 1
#ifdef USE_CUDA
#include <opencv2/cudaobjdetect.hpp>
#endif
#include <opencv2/opencv.hpp>

DEFINE_string(tty, "", "Path to Arduino device node. If not given, use fake.");
DEFINE_int32(webcam, 0,
             "Webcam# to use. Usually 0 for built-in, 1+ for external.");

constexpr char kFaceCascadeFile[] =
    "../haarcascades/haarcascade_frontalface_default.xml";
constexpr char kEyeCascadeFile[] = "../haarcascades/haarcascade_eye.xml";
constexpr char kMouthCascadeFile[] = "../haarcascades/haarcascade_smile.xml";
static const cv::Size kMinFaceSize(20, 20);
static const auto kMinTimeBetweenFire = std::chrono::seconds(5);
static const auto kFireTime = std::chrono::milliseconds(500);
static const int kMinConsecutiveOnTargetToFire = 10;

#define FIND_EYES 0

static const cv::Scalar kBlue(255, 0, 0);
static const cv::Scalar kGreen(0, 255, 0);
static const cv::Scalar kRed(0, 0, 255);
static const cv::Scalar kTeal(255, 255, 0);
static const cv::Scalar kYellow(0, 255, 255);
static const cv::Scalar kWhite(255, 255, 255);

static const cv::Point kImageSize(640, 480);
static const cv::Point kTarget(320, 240);
static const cv::Point kFovInSteps(200, 100);
static const int kTargetSize = 20;
static const int kMinStep = 4;

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

const char *kActionNames[] = {"LEFT", "RIGHT", "DOWN", "UP", "FIRE"};
struct Action {
  enum ActionEnum { LEFT, RIGHT, DOWN, UP, FIRE };

  Action(ActionEnum cmd, int steps) : cmd(cmd), steps(steps) {}

  ActionEnum cmd;
  int steps;
};

using time_point = std::chrono::time_point<std::chrono::high_resolution_clock>;
time_point now() { return std::chrono::high_resolution_clock::now(); }

void DoAction(Action action, Robot *robot) {
  switch (action.cmd) {
  case Action::LEFT:
    robot->left(action.steps);
    return;
  case Action::RIGHT:
    robot->right(action.steps);
    return;
  case Action::DOWN:
    robot->down(action.steps);
    return;
  case Action::UP:
    robot->up(action.steps);
    return;
  case Action::FIRE:
    robot->fire(kFireTime);
    return;
  }
}

std::ostream &operator<<(std::ostream &os, Action action) {
  return os << kActionNames[action.cmd] << "(" << action.steps << ")";
}

class Recognizer {
public:
#ifdef USE_CUDA
  using Mat = cv::cuda::GpuMat;
#else
  using Mat = cv::Mat;
#endif

  Recognizer(Robot *robot) : robot_(robot) {
#ifndef USE_CUDA
    QCHECK(face_detector_->load(kFaceCascadeFile))
        << " error loading " << kFaceCascadeFile;
#endif
  }

  static void PlotFeature(cv::Mat &mat, const cv::Rect &feature,
                          const cv::Scalar &color) {
    cv::rectangle(mat, feature.tl(), feature.br(), color);
  }

  static cv::Rect GuessMouthLocation(const cv::Rect &face) {
    return cv::Rect(
        cv::Point(face.x + face.width / 4, face.y + 9 * face.height / 12),
        cv::Size(face.width / 2, face.height / 6));
  }

  static cv::Rect BestMouth(const std::vector<cv::Rect> &mouths) {
    cv::Rect best = mouths[0];
    for (const cv::Rect &mouth : mouths)
      if (mouth.width * mouth.height > best.width * best.height)
        best = mouth;
    return best;
  }

  static cv::Rect BestFace(std::vector<cv::Rect> &faces) {
    cv::Rect best_face = faces[0];
    for (const cv::Rect& face : faces) {
      if (sqdist(Center(face), kTarget) < sqdist(Center(best_face), kTarget)) {
        best_face = face;
      }
    }
    return best_face;
  }

  std::vector<Action> DetermineAction(cv::Mat &input_img,
                                      const cv::Point &mouth) {
    QCHECK(input_img.rows == kImageSize.y);
    QCHECK(input_img.cols == kImageSize.x);
    constexpr int kTargetSize = 20;
    const cv::Rect target(kTarget - kTargetSize / 2,
                          cv::Size(kTargetSize, kTargetSize));
    PlotFeature(input_img, target, kTeal);
    auto vec = (kTarget - mouth) * kFovInSteps / kImageSize;
    std::vector<Action> actions;
    if (abs(vec.x) > 4) {
      actions.emplace_back(vec.x < 0 ? Action::RIGHT : Action::LEFT,
                           abs(vec.x));
      maybe_fire_ = 0;
    }
    if (abs(vec.y) > 4) {
      actions.emplace_back(vec.y < 0 ? Action::DOWN : Action::UP, abs(vec.y));
      maybe_fire_ = 0;
    }
    if (actions.empty()) {
      auto fire_time = now();
      if (++maybe_fire_ > kMinConsecutiveOnTargetToFire &&
          fire_time - last_fire_ > kMinTimeBetweenFire) {
        last_fire_ = fire_time;
        actions.emplace_back(Action::FIRE, 0);
        maybe_fire_ = 0;
      }
    }
    return actions;
  }

#ifdef USE_CUDA
  std::vector<cv::Rect> DetectMultiScale(cv::cuda::CascadeClassifier *cc,
                                         const cv::cuda::GpuMat &mat,
                                         double scale_factor = 1.3,
                                         int min_neighbors = 3,
                                         cv::Size min_size = {0, 0}) {
    std::vector<cv::Rect> rects;
    cv::cuda::GpuMat found;
    cc->setScaleFactor(scale_factor);
    cc->setMinNeighbors(min_neighbors);
    cc->setMinObjectSize(min_size);
    cc->detectMultiScale(mat, found);
    cc->convert(found, rects);
    return rects;
  }
#endif

  std::vector<cv::Rect> DetectMultiScale(cv::CascadeClassifier *cc,
                                         const cv::Mat &mat,
                                         double scale_factor = 1.3,
                                         int min_neighbors = 3,
                                         cv::Size min_size = {0, 0}) {
    std::vector<cv::Rect> rects;
    cc->detectMultiScale(mat, rects, scale_factor, min_neighbors, /*flags=*/0,
                         /*minSize=*/min_size);
    return rects;
  }

  void Detect(time_point timestamp, cv::Mat &input_img) {
    if (timestamp <= last_action_) {
      return;
    }
    cv::cvtColor(input_img, gray_, cv::COLOR_BGR2GRAY);
    std::ostringstream line1;
    std::ostringstream line2;
    auto faces =
        DetectMultiScale(face_detector_.get(), gray_, 1.3, 5, kMinFaceSize);
    for (const cv::Rect& face : faces) {
      PlotFeature(input_img, face, kBlue);
    }
    if (!faces.empty()) {
      auto face = BestFace(faces);
      cv::Rect mouth = GuessMouthLocation(face);
      PlotFeature(input_img, mouth, kYellow);
      for (Action action : DetermineAction(input_img, Center(mouth))) {
        DoAction(action, robot_);
        line2 << action << " ";
      }
      if (maybe_fire_ > 0) {
        line2 << "maybe_fire(" << maybe_fire_ << ")";
      }
    }
    last_action_ = now();
    line1 << "latency "
          << ((last_action_ - timestamp) / std::chrono::milliseconds(1))
          << " ms";
    cv::putText(input_img, line1.str(), cv::Point(0, 20),
                cv::FONT_HERSHEY_PLAIN, 1, kWhite);
    cv::putText(input_img, line2.str(), cv::Point(0, 40),
                cv::FONT_HERSHEY_PLAIN, 2, kWhite);
    cv::imshow("img", input_img);
  }

  Robot *robot() { return robot_; }

private:
  int maybe_fire_ = 0;
  Mat img_;
  Mat gray_;
  Robot *robot_;
  time_point last_action_;
  time_point last_fire_;
#ifdef USE_CUDA
  cv::Ptr<cv::cuda::CascadeClassifier> face_detector_ =
      cv::cuda::CascadeClassifier::create(kFaceCascadeFile);
#else
  std::unique_ptr<cv::CascadeClassifier> face_detector_{
      new cv::CascadeClassifier};
#endif
};

void DetectImages(Recognizer *recognizer, int argc, char **argv) {
  cv::Mat image;
  for (int i = 0; i < argc; ++i) {
    std::cout << "=== " << argv[i] << std::endl;
    image = cv::imread(argv[i], 1);
    if (!image.data) {
      std::cerr << "error reading image " << argv[i] << std::endl;
      continue;
    }
    recognizer->Detect(now(), image);
    if (cv::waitKey(0) == 'q')
      break;
  }
}

void DetectWebcam(Recognizer *recognizer) {
  std::mutex mu;
  std::condition_variable latest_image_cv;
  std::pair<time_point, cv::Mat> latest_image;
  std::atomic<bool> done(false);
  bool latest_image_ready = false;

  std::thread capture_thread([&] {
    cv::VideoCapture capture(FLAGS_webcam);
    QCHECK(capture.isOpened()) << "Failed to open --webcam=" << FLAGS_webcam;
    while (!done.load(std::memory_order_relaxed)) {
      cv::Mat image;
      if (!capture.grab()) {  // Defer decoding until after we calculate the timestamp.
        std::cerr << "error grabbing from --webcam=" << FLAGS_webcam
                  << std::endl;
      } else {
        time_point capture_time = now();
        if (!capture.retrieve(image)) {
          std::cerr << "error retrieving from --webcam=" << FLAGS_webcam
                    << std::endl;
        }
        std::unique_lock<std::mutex> lock(mu);
        latest_image.first = capture_time;
        latest_image.second = image;
        latest_image_ready = true;
        lock.unlock();
        latest_image_cv.notify_one();
      }
    }
  });

  std::thread detect_thread([&] {
    time_point timestamp;
    cv::Mat image;
    for (int key = 0; key != 'q';) {
      {
        std::unique_lock<std::mutex> lock(mu);
        latest_image_cv.wait(lock, [&] { return latest_image_ready; });
        std::tie(timestamp, image) = std::move(latest_image);
        latest_image_ready = false;
      }
      recognizer->Detect(timestamp, image);
      key = cv::waitKey(1000 / 30);
      while (key == 'p')
        key = cv::waitKey(0); // Wait for another key to be pressed.
      const int kManualMove = kFovInSteps.x / 4;
      switch (key) {
      case 'w':
        DoAction({Action::UP, kManualMove}, recognizer->robot());
        break;
      case 'a':
        DoAction({Action::LEFT, kManualMove}, recognizer->robot());
        break;
      case 's':
        DoAction({Action::DOWN, kManualMove}, recognizer->robot());
        break;
      case 'd':
        DoAction({Action::RIGHT, kManualMove}, recognizer->robot());
        break;
      case 'f':
        DoAction({Action::FIRE, 0}, recognizer->robot());
        break;
      }
    }
    done = true;
  });

  detect_thread.join();
  capture_thread.join();
}

int main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::unique_ptr<Robot> robot;
  if (!FLAGS_tty.empty()) {
    robot.reset(new RobotSerial(FLAGS_tty, 9600));
  } else {
    std::cerr
        << "warning: using fake robot. Provide --tty to connect to an arduino."
        << std::endl;
    robot.reset(new NoOpRobot());
  }
  Recognizer recognizer(robot.get());
  if (argc == 1) {
    DetectWebcam(&recognizer);
  } else {
    DetectImages(&recognizer, argc - 1, argv + 1);
  }
  return 0;
}
