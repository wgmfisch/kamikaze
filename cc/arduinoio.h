#include <iostream>
#include <string>
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

  void SendMessage(const std::string& command) {
    boost::asio::write(serial_port_,
                       boost::asio::buffer(Encode(command)));
  }

  void WriteOutput(Pin pin, bool value) {
    std::string command = "SET_IO";
    command.insert(command.end(), pin);
    command.insert(command.end(), value);
    SendMessage(command);
  }

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
  std::string Encode(const std::string& command) {
    const char address = 0;
    const char addr_size = 1;
    const char timeout = 0;
    std::ostringstream buf;
    buf << addr_size << char(command.size()) << timeout << address << command;
    char chk1 = 0;
    char chk2 = 0;
    for (char c : buf.str()) {
      chk1 += c;
      chk2 += chk1;
    }
    buf << chk2 << chk1;
    for (char c : buf.str()) {
      if (isprint(c)) std::cout << c;
      else std::cout << '\\' << uint16_t(uint8_t(c));
    }
    std::cout << std::endl;
    return buf.str();
  }

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
