#include "arduinoio.h"
#include "logging.h"

#include <iostream>
#include <string>

int main(int argc, char *argv[]) {
  QCHECK(argc > 1) << ": usage: " << argv[0] << " /dev/ttyACM0\n";
  ArduinoIO arduino(argv[1], 9600);
  bool value = true;
  while (true) {
    arduino.WriteOutput(13, value);
    sleep(1);
    value = !value;
  }

  Motor LR;
  LR.cmd.dir_pin = 3;
  LR.cmd.pulse_pin = 2;
  LR.cmd.pos_trigger_pin = 6;
  LR.cmd.neg_trigger_pin = ArduinoIO::kUnconnected3;
  LR.cmd.forward = true;
  LR.cmd.steps = 100;
  LR.ms1 = 14;
  LR.ms2 = 15;
  LR.ms3 = 16;

  Motor UD;
  UD.cmd.dir_pin = 3;
  UD.cmd.pulse_pin = 2;
  UD.cmd.pos_trigger_pin = 6;
  UD.cmd.neg_trigger_pin = ArduinoIO::kUnconnected3;
  UD.cmd.forward = true;
  UD.cmd.steps = 100;
  UD.ms1 = 8;
  UD.ms2 = 9;
  UD.ms3 = 10;

  bool forward = Motor::LEFT;
  while (true) {
    LR.Move(forward, 100, &arduino);
    sleep(1);
    forward = !forward;
  }
  return 0;
}
