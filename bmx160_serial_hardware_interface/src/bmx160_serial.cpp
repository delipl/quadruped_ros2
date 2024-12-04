// Copyright (c) 2024, Jakub Delicat
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "bmx160_serial_hardware_interface/bmx160_serial.hpp"
#include <fcntl.h>
#include <iostream>
#include <limits>
#include <regex>
#include <sstream>
#include <termios.h>
#include <unistd.h>

// Constructor: Opens the serial port
BMX160Serial::BMX160Serial(const std::string &port_name, int baud_rate)
    : port_name_{port_name}, baud_rate_{baud_rate} {}

bool BMX160Serial::initialize() {
  serial_fd = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_fd < 0) {
    std::cerr << "Error opening serial port!" << std::endl;
    return false;
  }

  struct termios tty;
  if (tcgetattr(serial_fd, &tty) != 0) {
    std::cerr << "Error from tcgetattr" << std::endl;
    return false;
  }

  cfsetospeed(&tty, baud_rate_);
  cfsetispeed(&tty, baud_rate_);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 1;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
    std::cerr << "Error from tcsetattr" << std::endl;
    return false;
  }
  return true;
}

// Destructor: Closes the serial port
BMX160Serial::~BMX160Serial() {
  if (serial_fd >= 0) {
    close(serial_fd);
  }
}

// Reads a line of input from the serial port
std::string BMX160Serial::read_line() {
  std::string line;
  char ch;
  while (read(serial_fd, &ch, 1) > 0 && ch != '\n') {
    if (ch != '\r') {
      line += ch;
    }
  }
  return line;
}

// Parses a single line of sensor data and updates the SensorData struct
void BMX160Serial::parse_line(const std::string &line, SensorData &data) {
  std::regex pattern(
      R"(([MAG]) X:\s*(-?\d+\.\d+)\s+Y:\s*(-?\d+\.\d+)\s+Z:\s*(-?\d+\.\d+)\s+(\S+))");
  std::smatch match;

  if (std::regex_search(line, match, pattern)) {
    char type = match[1].str()[0];
    float x = std::stof(match[2].str());
    float y = std::stof(match[3].str());
    float z = std::stof(match[4].str());

    switch (type) {
    case 'M':
      data.mag_x = x;
      data.mag_y = y;
      data.mag_z = z;
      break;
    case 'G':
      data.gyro_x = x;
      data.gyro_y = y;
      data.gyro_z = z;
      break;
    case 'A':
      data.accel_x = x;
      data.accel_y = y;
      data.accel_z = z;
      break;
    }
  }
}

// Reads and returns the latest sensor data
BMX160Serial::SensorData BMX160Serial::read_sensor_data() {
  const auto nan = std::numeric_limits<float>::quiet_NaN();
  SensorData data = {nan, nan, nan, nan, nan, nan, nan, nan, nan};

  for (int i = 0; i < 3; i++) { // Expecting three lines: M, G, A
    std::string line = read_line();
    parse_line(line, data);
  }

  return data;
}
