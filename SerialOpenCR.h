#include <cstdint>
#include <string>
#include <termios.h>

#include "SerialMessageObj.h"
#include "include/current_command.h"
#include "include/velocity_command.h"
#include "include/position_command.h"
#include "include/imu_status.h"
#include "include/motor_status.h"

struct SerialOpenCR {
	enum MessageTypes : uint8_t {
		OCR_MESSAGE = 0,
		IMU_STATUS,
		MOTOR_STATUS,
		CTRL_MODE,
		CUR_COMMAND,
		VEL_COMMAND,
		POS_COMMAND
	};
	enum ControlModes : uint8_t { CUR_CTRL = 0, VEL_CTRL = 1, POS_CTRL = 4 };
	SerialMessage serial_message;
	std::string path;
	int serialPort = -1;
	struct termios tty;

	MotorStatus motor_status;
	double averageMotorDelta = 0;
	ImuStatus imu_status;
	double averageIMUDelta = 0;
	std::string text_status;

	bool waitForDevice();

	bool setupConnection();

	void WriteMessage(const char* message);
	void WriteCtrlModeCommand(const ControlModes& control_mode);
	void WriteCurrentCommand(const CurrentCommand& command);
	void WriteVelocityCommand(const VelocityCommand& command);
	void WritePositionCommand(const PositionCommand& command);

	void ReadMotorStatus(const uint8_t* package);
	void ReadImuStatus(const uint8_t* package);
	void ReadTextStatus(const uint8_t* message, size_t length);

	SerialOpenCR(const std::string path);
};
