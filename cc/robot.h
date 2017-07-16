#include <stdio.h>

#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "arduinoio.h"

class Robot {
public:
  virtual ~Robot() {}
  virtual void up(int steps) = 0;
  virtual void down(int steps) = 0;
  virtual void left(int steps) = 0;
  virtual void right(int steps) = 0;
  virtual void fire() = 0;
};

class RobotSerial final : public Robot {
public:
  RobotSerial(const std::string& tty, int baud);
  ~RobotSerial() final;

  void up(int steps) final { UD_.Move(Motor::UP, steps, &io_); }
  void down(int steps) final { UD_.Move(Motor::DOWN, steps, &io_); }
  void left(int steps) final { LR_.Move(Motor::LEFT, steps, &io_); }
  void right(int steps) final { LR_.Move(Motor::RIGHT, steps, &io_); }
  void fire() final {
    valve(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    valve(false);
  }

private:
  void valve(bool open) {
    io_.WriteOutput(11, open);
    io_.WriteOutput(13, open);
  }
  ArduinoIO io_;
  Motor LR_;
  Motor UD_;
};

class NoOpRobot : public Robot {
public:
  void up(int) final {}
  void down(int) final {}
  void left(int) final {}
  void right(int) final {}
  void fire() final {}
};
