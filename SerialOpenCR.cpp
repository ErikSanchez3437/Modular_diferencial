#include "SerialOpenCR.h"
#include "SerialMessage.h"
#include "include/velocity_command.h"

#include <cstring>

#include <cassert> // assert
#include <cerrno>  // Error integer and strerror() function
#include <fcntl.h> // Contains file controls like O_RDWR
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

constexpr size_t MESSAGE_BUF_SIZE = 80;
void SerialOpenCR::WriteMessage(const char* message) {
	using std::min;
	uint8_t buf[MESSAGE_BUF_SIZE + 1];
	size_t len     = strlen(message);
	size_t written = 0;
	buf[0]         = OCR_MESSAGE;
	while (written < len) {
		size_t toWrite = min(len - written, size_t(MESSAGE_BUF_SIZE));
		memcpy(buf + 1, message + written, toWrite);

		serial_message.writePackage(buf, toWrite + 1);
		written += toWrite;
	}
}

constexpr size_t CTRL_MODE_BUF_SIZE = 1;
void SerialOpenCR::WriteCtrlModeCommand(const ControlModes& ctrl_mode) {
	uint8_t buf[CTRL_MODE_BUF_SIZE + 1];
	buf[0]                = CTRL_MODE;
	uint16_t writtenBytes = 1;
	memcpy(buf + writtenBytes, &ctrl_mode, sizeof(ctrl_mode));             \
	writtenBytes += sizeof(ctrl_mode);
	serial_message.writePackage(buf, writtenBytes);
}

constexpr size_t CUR_BUF_SIZE = 4;
void SerialOpenCR::WriteCurrentCommand(const CurrentCommand& command) {
	uint8_t buf[CUR_BUF_SIZE + 1];
	buf[0]                = CUR_COMMAND;
	uint16_t writtenBytes = 1;
#define WRITE_MEM(mem)                                                         \
	memcpy(buf + writtenBytes, &command.mem, sizeof(command.mem));             \
	writtenBytes += sizeof(command.mem);
	WRITE_MEM(lCur);
	WRITE_MEM(rCur);
#undef WRITE_MEM
	serial_message.writePackage(buf, writtenBytes);
}

constexpr size_t VEL_BUF_SIZE = 8;
void SerialOpenCR::WriteVelocityCommand(const VelocityCommand& command) {
	uint8_t buf[VEL_BUF_SIZE + 1];
	buf[0]                = VEL_COMMAND;
	uint16_t writtenBytes = 1;
#define WRITE_MEM(mem)                                                         \
	memcpy(buf + writtenBytes, &command.mem, sizeof(command.mem));             \
	writtenBytes += sizeof(command.mem);
	WRITE_MEM(lVel);
	WRITE_MEM(rVel);
#undef WRITE_MEM
	serial_message.writePackage(buf, writtenBytes);
}

constexpr size_t POS_BUF_SIZE = 8;
void SerialOpenCR::WritePositionCommand(const PositionCommand& command) {
	uint8_t buf[POS_BUF_SIZE + 1];
	buf[0]                = POS_COMMAND;
	uint16_t writtenBytes = 1;
#define WRITE_MEM(mem)                                                         \
	memcpy(buf + writtenBytes, &command.mem, sizeof(command.mem));             \
	writtenBytes += sizeof(command.mem);
	WRITE_MEM(lPos);
	WRITE_MEM(rPos);
#undef WRITE_MEM
	serial_message.writePackage(buf, writtenBytes);
}

void SerialOpenCR::ReadTextStatus(const uint8_t* message, size_t length) {
	text_status = std::string(message, message + length);
}

double averageIMUDelta        = 0;
constexpr size_t IMU_BUF_SIZE = 20;
void SerialOpenCR::ReadImuStatus(const uint8_t* package) {
	size_t readBytes = 0;
#define READ_MEM(mem)                                                          \
	memcpy(&imu_status.mem, package + readBytes, sizeof(imu_status.mem));      \
	readBytes += sizeof(imu_status.mem);
	READ_MEM(delta);
	READ_MEM(roll);
	READ_MEM(pitch);
	READ_MEM(yaw);
	READ_MEM(accX);
	READ_MEM(accY);
	READ_MEM(accZ);
#undef READ_MEM
	averageIMUDelta = averageIMUDelta * 0.95 + imu_status.delta * 0.05;
}

constexpr size_t MOTOR_BUF_SIZE = 22;
void SerialOpenCR::ReadMotorStatus(const uint8_t* package) {
	size_t readBytes = 0;
#define READ_MEM(mem)                                                          \
	memcpy(&motor_status.mem, package + readBytes, sizeof(motor_status.mem));  \
	readBytes += sizeof(motor_status.mem);
	READ_MEM(delta);
	READ_MEM(lCur);
	READ_MEM(rCur);
	READ_MEM(lVel);
	READ_MEM(rVel);
	READ_MEM(lPos);
	READ_MEM(rPos);
#undef READ_MEM
	averageMotorDelta = averageMotorDelta * 0.95 + motor_status.delta * 0.05;
}

void packageCall(uint8_t* data, uint32_t size, void* u_data) {
	SerialOpenCR& serial = *(SerialOpenCR*)u_data;
	if (size) {
		switch (*data) {
		case SerialOpenCR::MessageTypes::OCR_MESSAGE:
			serial.ReadTextStatus(data + 1, size - 1);
			return;
		case SerialOpenCR::MessageTypes::IMU_STATUS:
			if (size == IMU_BUF_SIZE + 1)
				serial.ReadImuStatus(data + 1);
			return;
		case SerialOpenCR::MessageTypes::MOTOR_STATUS:
			if (size == MOTOR_BUF_SIZE + 1)
				serial.ReadMotorStatus(data + 1);
			return;
		}
	}
}

uint32_t availToWriteCall(void* u_data) {
	int res;
	ioctl(*(int*)u_data, TIOCOUTQ, &res);
	return 256 - bool(res) * 256;
}

uint32_t writeCall(uint8_t* data, uint32_t size, void* u_data) {
	auto res = write(*(int*)u_data, data, size);
	if (res < 0) {
		printf("Error %i from write: %s\n", errno, strerror(errno));
	}
	assert(res >= 0);
	// std::cout << "write called: " << res << std::endl;
	return res;
}

uint32_t availToReadCall(void* u_data) {
	int res;
	ioctl(*(int*)u_data, FIONREAD, &res);
	return res;
}

uint32_t readCall(uint8_t* data, uint32_t size, void* u_data) {
	auto res = read(*(int*)u_data, data, size);
	if (res < 0) {
		printf("Error %i from read: %s\n", errno, strerror(errno));
	}
	assert(res >= 0);
	// std::cout << "read called: " << res << std::endl;
	return res;
}

SerialOpenCR::SerialOpenCR(const std::string path) : path(path) {
}

bool SerialOpenCR::waitForDevice() {
	while (serialPort < 0) {
		serialPort = open(path.c_str(), O_RDWR);
	}
	return true;
}

bool SerialOpenCR::setupConnection() {
	if (serialPort < 0) {
		return false;
	}
	if (tcgetattr(serialPort, &tty) != 0) {
		printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
		return false;
	}

	tty.c_cflag &= ~PARENB;  // Clear parity bit, disabling parity (most common)
	tty.c_cflag &= ~CSTOPB;  // Clear stop field, only one stop bit
	tty.c_cflag &= ~CSIZE;   // Clear all the size bits
	tty.c_cflag |= CS8;      // 8 bits per byte
	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines

	tty.c_lflag &= ~ICANON; // Disable canonical mode
	tty.c_lflag &= ~ECHO;   // Disable echo
	tty.c_lflag &= ~ECHOE;  // Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); /* no SW flowcontrol */
	tty.c_iflag &= ~(INPCK | IUCLC | IMAXBEL);

	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline

	tty.c_cc[VTIME] = 0; // Don't block
	tty.c_cc[VMIN]  = 0;
	tty.c_iflag &= ~(INLCR | IGNCR | ICRNL);
	// Set in/out baud rate to be 115200
	cfsetispeed(&tty, B1152000);
	cfsetospeed(&tty, B1152000);

	// Save tty settings, also checking for error
	if (tcsetattr(serialPort, TCSANOW, &tty) != 0) {
		printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
		return false;
	}
	serial_struct serial;
	ioctl(serialPort, TIOCGSERIAL, &serial);
	serial.flags |= ASYNC_LOW_LATENCY;
	ioctl(serialPort, TIOCSSERIAL, &serial);
	tcflush(serialPort, TCIOFLUSH);

	serial_message.setPackageUserData(this);
	serial_message.setPackageCallback(packageCall);

	serial_message.setAvailToWriteUserData(&serialPort);
	serial_message.setAvailToWriteCallback(availToWriteCall);

	serial_message.setWriteUserData(&serialPort);
	serial_message.setWriteCallback(writeCall);

	serial_message.setAvailToReadUserData(&serialPort);
	serial_message.setAvailToReadCallback(availToReadCall);

	serial_message.setReadUserData(&serialPort);
	serial_message.setReadCallback(readCall);
	return true;
}
