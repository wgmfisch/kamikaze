#include <iostream>
#include <string>
#include <thread>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>


class ArduinoIO {
public:
  using Pin = char;
  static constexpr Pin kUnconnected1 = 17;
  static constexpr Pin kUnconnected2 = 18;
  static constexpr Pin kUnconnected3 = 12;

  ArduinoIO(const std::string& port, int baud_rate) {
    serial_port_.open(port);
    serial_port_.set_option(
        boost::asio::serial_port_base::baud_rate(baud_rate));
  }

  void WriteOutput(Pin pin, bool value);

  struct MoveCmd {
    Pin dir_pin;
    Pin pulse_pin;
    Pin neg_trigger_pin;
    Pin pos_trigger_pin;
    Pin done_pin;
    bool forward;
    uint8_t max_wait = 0;
    int32_t steps; // little-endian -- fine on x86.
    int32_t temp_pin_threshold = 0;  // little-endian -- fine on x86.
  };

  void Move(const MoveCmd& cmd) {
    SendMessage("MOVE" + std::string((const char*) &cmd, sizeof(cmd)));
  }

  ~ArduinoIO() {
    serial_port_.close();
  }

private:
  void SendMessage(std::string command);

  boost::asio::io_service io_;
  boost::asio::serial_port serial_port_{io_};
};

struct Motor {
  ArduinoIO::MoveCmd cmd;

  ArduinoIO::Pin ms1;
  ArduinoIO::Pin ms2;
  ArduinoIO::Pin ms3;

  enum : bool { RIGHT = 0, LEFT = 1, UP = 0, DOWN = 1 };

  void Init(ArduinoIO* io) {
    io->WriteOutput(ms1, 1);
    io->WriteOutput(ms2, 1);
    io->WriteOutput(ms3, 1);
  }

  void Move(bool forward, int steps, ArduinoIO* io) {
    cmd.forward = forward;
    cmd.steps = steps;
    io->Move(cmd);
  }
};
