// C library headers
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <ratio>
#include <stdio.h>
#include <string.h>
#include <thread>
#include <vector>

#include "include/imu_status.h"
#include "include/motor_status.h"

// Linux headers
#include <errno.h> // Error integer and strerror() function
#include <fcntl.h> // Contains file controls like O_RDWR
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

#include "LockFreeLatest.h"

//
#define SERIAL_MESSAGE_IMPL
//#include "SerialOpenCR.h"
#include "SerialOpenCR.cpp"
//
//

std::chrono::steady_clock::time_point start;

int main() {
	SerialOpenCR openCR("/dev/ttyACM0");
	openCR.waitForDevice();

	std::thread printingThread([&]() {
		while (true) {
			std::cout << "\033[2J\033[1;1H";
			std::this_thread::sleep_for(std::chrono::milliseconds(16));
			float ft = std::chrono::duration<float>(
			               std::chrono::steady_clock::now() - start)
			               .count();
			char buffer[40] = {};
			sprintf(buffer, "time %10f\n", ft);
			std::cout << std::string(buffer);
			// printf("<< %10f\n", ft);
			std::cout << "latestMessage:" << openCR.text_status
								<< std::endl;
			//auto& imuLatest = imuStatus.getLatest();
			auto& imuLatest = openCR.imu_status;
			std::cout << ">> average delta " << openCR.averageIMUDelta << std::endl;
			//std::cout << ">> sample delta " << imuLatest.sample_delta
								//<< std::endl;
			std::cout << ">> roll " << imuLatest.roll << std::endl;
			std::cout << ">> pitch " << imuLatest.pitch << std::endl;
			std::cout << ">> yaw " << imuLatest.yaw << std::endl;
			std::cout << ">> acc.x " << imuLatest.accX * 2.0 / 32768.0
			          << std::endl;
			std::cout << ">> acc.y " << imuLatest.accY * 2.0 / 32768.0
			          << std::endl;
			std::cout << ">> acc.z " << imuLatest.accZ * 2.0 / 32768.0
			          << std::endl;
			/*
			 *std::cout << ">> mag.x " << imuLatest.mag[0] * 10. * 4912. / 32760.
			 *          << std::endl;
			 *std::cout << ">> mag.y " << imuLatest.mag[1] * 10. * 4912. / 32760.
			 *          << std::endl;
			 *std::cout << ">> mag.z " << imuLatest.mag[2] * 10. * 4912. / 32760.
			 *          << std::endl;
			 */

			//auto& status = motorStatus.getLatest();
			auto& status = openCR.motor_status;
			std::cout << ">> average delta " << openCR.averageMotorDelta << std::endl;
			std::cout << ">> sample delta " << status.delta << std::endl;
			std::cout << ">> leftCur " << status.lCur << std::endl;
			std::cout << ">> rightCur " << status.rCur << std::endl;
			std::cout << ">> leftVel " << status.lVel << std::endl;
			std::cout << ">> rightVel " << status.rVel << std::endl;
			std::cout << ">> leftPos " << status.lPos << std::endl;
			std::cout << ">> rightPos " << status.rPos << std::endl;
		}
	});
	if(!openCR.setupConnection()) return 1;
	

	auto lastSent = std::chrono::steady_clock::now();
	start         = std::chrono::steady_clock::now();
	openCR.WriteCtrlModeCommand(SerialOpenCR::CUR_CTRL);
	while (true) {

		// std::cout << buffer << std::endl;
		auto now        = std::chrono::steady_clock::now();
		float sendDelta = 0;
		bool vel = 0;
		{ sendDelta = std::chrono::duration<float>(now - lastSent).count(); }
		if (sendDelta >= 0.001) {
			lastSent        = now;
			char buffer[40] = {};
			float ft        = std::chrono::duration<float>(now - start).count();
			sprintf(buffer, "time %10f\n", ft);
			//openCR.WriteMessage(buffer);
			// printf("<< %10f\n", ft);
			//openCR.WriteVelocityCommand({0x02020202,0x02020202});
			//openCR.WriteCurrentCommand({0xff,0xff});
//			openCR.WritePositionCommand({345,345});
			//openCR.WriteVelocityCommand({-200,200});
			openCR.WriteCurrentCommand({-10,10});
		};
		openCR.serial_message.poll();
	}
}
