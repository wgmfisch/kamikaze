#include "robot.h"
#include "logging.h"

#include <stdio.h>
#include <memory.h>

ProxyRobot::ProxyRobot(const char *tty) {
  char buffer[100];
  sprintf(buffer, "../robot_proxy.py %s", tty);
  sprintf(buffer, "date %s", tty);
  QCHECK(subproc = popen(buffer, "rw")) << ": cmd=" << buffer;
  std::cout << "Waiting for robot_proxy.py to be ready...." << std::endl;
  do {
    fgets(buffer, sizeof(buffer) - 1, subproc);
    if (strcmp(buffer, "robot_proxy.py: ready\n") == 0) {
      break;
    }
    std::cerr << "robot_proxy.py babbled: " << buffer << std::endl;
    QCHECK(!feof(subproc)) << "robot_proxy.py exited while intializing";
  } while (true);
  std::cout << "robot_proxy.py is ready." << std::endl;
}

ProxyRobot::~ProxyRobot() {
  pclose(subproc);
}

bool ProxyRobot::command_sync(const char *c, int n) {
  char buffer[100];
  sprintf(buffer, "%s %d", c, n);
  return command_sync(buffer);
}

bool ProxyRobot::command_sync(const char *c) {
  char buffer[100];
  std::cout << "sent " << c << ", waiting for ack..." << std::endl;
  do {
    fgets(buffer, sizeof(buffer) - 1, subproc);
    if (strcmp(buffer, "done") == 0) {
      break;
    }
    std::cerr << "robot_proxy.py babbled: " << buffer << std::endl;
    QCHECK(!feof(subproc)) << "robot_proxy.py exited while running " << c;
  } while(true);
}
