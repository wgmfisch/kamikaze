#!/usr/bin/env python
import sys
import robot


def send_cmd(cmd, robot):
  cmd = cmd.strip().split()
  if not cmd:
    return
  getattr(robot, cmd[0])(*map(int, cmd[1:]))


def read_loop(robot):
  print 'robot_proxy.py: ready'
  try:
    while True:
      send_cmd(raw_input(), robot)
  except EOFError:
    pass


if __name__ == "__main__":
  if len(sys.argv) < 2:
    print 'robot_proxy.py: warning: using fake'
    read_loop(robot.LoudFakeRobot())
  else:
    read_loop(robot.Robot(sys.argv[1]))
