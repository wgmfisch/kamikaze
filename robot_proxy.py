#!/usr/bin/env python
import sys
import robot

OUTPUT_FILE = 'reply_from_proxy'


def reply(msg):
  with open(OUTPUT_FILE, 'w') as f:
    print >> f, msg


def send_cmd(cmd, robot):
  cmd = cmd.strip().split()
  if not cmd:
    return
  getattr(robot, cmd[0])(*map(int, cmd[1:]))


def read_loop(robot):
  reply('ready')
  i = 0
  try:
    while True:
      cmd = raw_input()
      send_cmd(cmd, robot)
      reply('%i %s' % (i, cmd))
      i += 1
  except EOFError:
    pass


if __name__ == "__main__":
  if len(sys.argv) < 2:
    print 'robot_proxy.py: warning: using fake'
    read_loop(robot.LoudFakeRobot())
  else:
    print 'robot_proxy.py: connecting to', sys.argv[1]
    read_loop(robot.Robot(sys.argv[1]))
