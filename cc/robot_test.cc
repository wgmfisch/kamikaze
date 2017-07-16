#include "robot.h"
#include "logging.h"

#include <iostream>
#include <string>

int main(int argc, char *argv[]) {
  QCHECK(argc > 1) << ": usage: " << argv[0] << " /dev/ttyACM0\n";

  RobotSerial robot(argv[1], 9600);
  while (true) {
    robot.fire();
    sleep(1);
    robot.left(100);
    sleep(1);
    robot.right(100);
    sleep(1);
    robot.up(100);
    sleep(1);
    robot.down(100);
  }
  return 0;
}
