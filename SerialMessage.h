#ifndef SERIAL_MESSAGE_H
#define SERIAL_MESSAGE_H
#include <cstdint>

uint32_t WriteSerialPackage(const uint8_t* data, uint32_t size);

void setPackageUserData(void* newData);
void setPackageCallback(void (*)(uint8_t*, uint32_t, void*));

void setAvailToWriteUserData(void* newData);
void setAvailToWriteCallback(uint32_t (*)(void*));

void setWriteUserData(void* newData);
void setWriteCallback(uint32_t (*)(uint8_t*, uint32_t, void*));

void setAvailToReadUserData(void* newData);
void setAvailToReadCallback(uint32_t (*)(void*));

void setReadUserData(void* newData);
void setReadCallback(uint32_t (*)(uint8_t*, uint32_t, void*));

void pollSerial();
#ifdef SERIAL_MESSAGE_IMPL
#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <cstring>

using std::min;

const uint8_t startBytes[5]  = {'t', 'u', 'r', 'l', '\n'};
const uint8_t middleBytes[1] = {'b'};

char read_buf[256 * 256];
size_t readTail;
size_t readHead;
char send_buf[256 * 256];
size_t writeTail;
size_t writeHead;
uint8_t current_package[256];
constexpr int writeAttempts = 8;

enum PackageStatus : int {
	START    = 0,
	LENGTH   = 1,
	MIDDLE   = 2,
	DATA     = 3,
	CHECKSUM = 4
};

struct ReadStatus {
	PackageStatus status      = START;
	uint32_t in_part          = 0;
	uint32_t in_data          = 0;
	uint32_t in_start         = 0;
	uint32_t package_len      = 0;
	uint32_t checksum         = 0;
	uint32_t package_checksum = 0;
} readStatus;

void* availToWriteUserData;
uint32_t (*availToWriteCallback)(void*) = nullptr;

void* writeUserData;
uint32_t (*writeCallback)(uint8_t*, uint32_t, void*) = nullptr;

void* availToReadUserData;
uint32_t (*availToReadCallback)(void*) = nullptr;

void* readUserData;
uint32_t (*readCallback)(uint8_t*, uint32_t, void*) = nullptr;

void* packageUserData;
void (*packageCallback)(uint8_t*, uint32_t, void*) = nullptr;

void setAvailToWriteUserData(void* newData) {
	availToWriteUserData = newData;
}
void setAvailToWriteCallback(uint32_t (*newAvailCallback)(void*)) {
	availToWriteCallback = newAvailCallback;
}

void setWriteUserData(void* newData) {
	writeUserData = newData;
}
void setWriteCallback(uint32_t (*newWriteCallback)(uint8_t*, uint32_t, void*)) {
	writeCallback = newWriteCallback;
}

void setAvailToReadUserData(void* newData) {
	availToReadUserData = newData;
}
void setAvailToReadCallback(uint32_t (*newAvailCallback)(void*)) {
	availToReadCallback = newAvailCallback;
}

void setReadUserData(void* newData) {
	readUserData = newData;
}
void setReadCallback(uint32_t (*newReadCallback)(uint8_t*, uint32_t, void*)) {
	readCallback = newReadCallback;
}

void setPackageUserData(void* newData) {
	packageUserData = newData;
}
void setPackageCallback(void (*newPackageCallback)(uint8_t*, uint32_t, void*)) {
	packageCallback = newPackageCallback;
}

uint32_t toWriteBytes() {
	return writeTail > writeHead ? sizeof(send_buf) - writeTail
	                             : writeHead - writeTail;
}

uint32_t toReadBytes() {
	return readTail > readHead ? sizeof(read_buf) - readTail + readHead
	                           : readHead - readTail;
}

uint32_t writeAvailBytes() {
	return writeTail > writeHead
	           ? writeTail - writeHead - 1
	           : sizeof(send_buf) - writeHead - (writeTail == 0);
}

uint32_t readAvailBytes() {
	return readTail > readHead ? readTail - readHead - 1
	                           : sizeof(read_buf) - readHead - (readTail == 0);
}

void readPending() {
	char curr = read_buf[readTail];
	++readTail;

	if (readTail == sizeof(read_buf)) {
		readTail = 0;
	}
	bool reset    = false;
	bool consumed = false;

	if (curr == startBytes[readStatus.in_start]) {
		readStatus.in_start++;
		if (readStatus.in_start == sizeof(startBytes)) {
			readStatus.status           = LENGTH;
			readStatus.in_part          = 0;
			readStatus.in_start         = 0;
			readStatus.in_data          = 0;
			readStatus.package_len      = 0;
			readStatus.checksum         = 0;
			readStatus.package_checksum = 0;
			consumed                    = true;
		}
	} else {
		readStatus.in_start = curr == startBytes[0];
	}

	if (!consumed) {
		switch (readStatus.status) {
		case LENGTH:
			((char*)&readStatus.package_len)[readStatus.in_part] = curr;
			if (readStatus.package_len > sizeof(current_package))
				reset = true;
			readStatus.in_part++;
			if (readStatus.in_part == sizeof(uint32_t)) {
				readStatus.checksum ^= readStatus.package_len;
				readStatus.status  = MIDDLE;
				readStatus.in_part = 0;
				readStatus.in_data = 0;
			}
			break;
		case MIDDLE:
			if (curr == middleBytes[readStatus.in_part]) {
				readStatus.in_part++;
			} else {
				reset = true;
			}
			if (readStatus.in_part == sizeof(middleBytes)) {
				readStatus.status = readStatus.in_data == readStatus.package_len
				                        ? CHECKSUM
				                        : DATA;
				readStatus.in_part = 0;
			}
			break;
		case DATA:
			current_package[readStatus.in_data] = curr;
			readStatus.in_data++;
			readStatus.in_part++;
			if (readStatus.in_part == sizeof(uint32_t)) {
				readStatus.checksum ^=
				    *(uint32_t*)(current_package + readStatus.in_data -
				                 readStatus.in_part);
				readStatus.status  = MIDDLE;
				readStatus.in_part = 0;
			} else if (readStatus.in_data == readStatus.package_len) {
				uint32_t finalInt = 0;
				memcpy(&finalInt,
				       current_package + readStatus.in_data -
				           readStatus.in_part,
				       readStatus.in_part);
				readStatus.checksum ^= finalInt;
				readStatus.status  = MIDDLE;
				readStatus.in_part = 0;
			}
			break;
		case CHECKSUM:
			((char*)&readStatus.package_checksum)[readStatus.in_part] = curr;
			readStatus.in_part++;
			if (readStatus.in_part == sizeof(uint32_t)) {
				reset = true;
				if (readStatus.checksum == readStatus.package_checksum) {
					packageCallback(current_package, readStatus.package_len,
					                packageUserData);
				} else {
				}
			}
		}
	}
	if (reset) {
		readStatus.status           = START;
		readStatus.in_part          = 0;
		readStatus.in_start         = 0;
		readStatus.in_data          = 0;
		readStatus.package_len      = 0;
		readStatus.checksum         = 0;
		readStatus.package_checksum = 0;
	}
}

void readAvailable() {
	uint32_t availToRead = availToReadCallback(availToReadUserData);
	if (availToRead > 0) {
		uint32_t bytes =
		    readCallback((uint8_t*)&read_buf + readHead,
		                 min(availToRead, readAvailBytes()), readUserData);
		readHead += bytes;
		if (readHead == sizeof(read_buf)) {
			readHead = 0;
		}
	}
}

uint32_t writePending() {
	uint32_t toWrite = toWriteBytes();
	if (toWrite) {
		uint32_t availToWrite = availToWriteCallback(availToWriteUserData);
		if (availToWrite > 0) {
			uint32_t bytes =
			    writeCallback((uint8_t*)&send_buf + writeTail,
			                  min(availToWrite, toWrite), writeUserData);
			if (bytes > 0) {
				writeTail += bytes;
				if (writeTail == sizeof(send_buf)) {
					writeTail = 0;
				}
				return bytes;
			}
		}
	}
	return 0;
}

void writeToBuffer(const uint8_t* data, uint32_t size) {
	uint32_t written = 0;
	while (written < size) {
		uint32_t availToWrite;
		uint32_t i = 0;
		while (!(availToWrite = writeAvailBytes())) {
			if (i >= writeAttempts) {
				writeTail += min(size - written, toWriteBytes());
				if (writeTail == sizeof(send_buf)) {
					writeTail = 0;
				}
				break;
			}
			if (!writePending()) {
				i++;
			}
		}
		if (availToWrite) {
			uint32_t toCopy = min(size - written, availToWrite);
			memcpy((uint8_t*)&send_buf + writeHead, data + written, toCopy);
			writeHead += toCopy;
			written += toCopy;
			if (writeHead == sizeof(send_buf)) {
				writeHead = 0;
			}
		}
	}
}

uint32_t WriteSerialPackage(const uint8_t* data, uint32_t size) {
	uint32_t toWrite    = size / 4;
	uint32_t extraWrite = size % 4;
	uint32_t checkSum  = 0;
	writeToBuffer(startBytes, sizeof(startBytes));
	checkSum ^= *(uint32_t*)&size;
	writeToBuffer((uint8_t*)&size, sizeof(uint32_t));
	for (uint32_t i = 0; i < toWrite; i++) {
		checkSum ^= ((uint32_t*)data)[i];
		writeToBuffer(middleBytes, sizeof(middleBytes));
		writeToBuffer(data + i * sizeof(uint32_t), sizeof(uint32_t));
	}
	if (extraWrite) {
		writeToBuffer(middleBytes, sizeof(middleBytes));
		writeToBuffer(data + toWrite * sizeof(uint32_t), extraWrite);
		uint32_t finalInt = 0;
		memcpy(&finalInt, data + toWrite * sizeof(uint32_t), extraWrite);
		checkSum ^= finalInt;
	}
	writeToBuffer(middleBytes, sizeof(middleBytes));
	writeToBuffer((uint8_t*)&checkSum, sizeof(uint32_t));
	return size;
}

void pollSerial() {

	readAvailable();
	while (toReadBytes()) {
		readPending();
	}
	writePending();
}
#endif
#endif
