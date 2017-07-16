#include "robot.h"

RobotSerial::RobotSerial(const std::string &tty, int baud) : io_(tty, baud) {
  valve(false);

  LR_.cmd.dir_pin = 3;
  LR_.cmd.pulse_pin = 2;
  LR_.cmd.pos_trigger_pin = 6;
  LR_.cmd.neg_trigger_pin = ArduinoIO::kUnconnected3;
  LR_.cmd.forward = true;
  LR_.cmd.steps = 100;
  LR_.ms1 = 14;
  LR_.ms2 = 15;
  LR_.ms3 = 16;

  UD_.cmd.dir_pin = 4;
  UD_.cmd.pulse_pin = 5;
  UD_.cmd.pos_trigger_pin = ArduinoIO::kUnconnected3;
  UD_.cmd.neg_trigger_pin = 7;
  UD_.cmd.forward = true;
  UD_.cmd.steps = 100;
  UD_.ms1 = 8;
  UD_.ms2 = 9;
  UD_.ms3 = 10;

  LR_.Init(&io_);
  UD_.Init(&io_);
}

RobotSerial::~RobotSerial() {}
