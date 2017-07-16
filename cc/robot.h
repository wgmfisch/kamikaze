#include <stdio.h>

#include <iostream>
#include <string>
#include <vector>

class Robot {
  public:
    virtual ~Robot() {}
    virtual bool up(int steps)  = 0;
    virtual bool down(int steps)  = 0;
    virtual bool left(int steps)  = 0;
    virtual bool right(int steps)  = 0;
    virtual bool calibrate()  = 0;
    virtual bool fire()  = 0;
};

class ProxyRobot final : public Robot {
  public:
    ProxyRobot(const char *tty);
    ~ProxyRobot() final;

    bool up(int steps) final { return command_sync("up", steps); }
    bool down(int steps) final { return command_sync("down", steps); }
    bool left(int steps) final { return command_sync("left", steps); }
    bool right(int steps) final { return command_sync("right", steps); }
    bool calibrate() final { return command_sync("calibrate"); }
    bool fire() final { return command_sync("fire"); }

  private:
    bool command_sync(const char *c, int n);
    bool command_sync(const char *c);

    FILE* subproc;
};

class NoOpRobot : public Robot {
  public:
    bool up(int) final { return true; }
    bool down(int) final { return true; }
    bool left(int) final { return true; }
    bool right(int) final { return true; }
    bool calibrate() final { return true; }
    bool fire() final { return true; }
};
