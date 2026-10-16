#define SERIAL_MESSAGE_IMPL
#include "SerialMessage.h"
#include <variant.h>
#include <DynamixelSDK.h>
#include <IMU.h>
//#include <usbd_cdc_interface.h>


long lastSent;

cIMU IMU;
enum MessageTypes : uint8_t {
  OCR_MESSAGE = 0,
  IMU_STATUS,
  MOTOR_STATUS,
  CTRL_MODE,
  CUR_COMMAND,
  VEL_COMMAND,
  POS_COMMAND
};

struct ImuStatus {
  uint16_t delta; /**< The time between samples, in &micro;s. */
  float roll;     /**< The roll measured by the IMU, in degrees. */
  float pitch;    /**< The pitch measured by the IMU, in degrees. */
  float yaw;      /**< The yaw measured by the IMU, in degrees. */
  int16_t accX;   /**< The x acceleration measured by the IMU. */
  int16_t accY;   /**< The y acceleration measured by the IMU. */
  int16_t accZ;   /**< The z acceleration measured by the IMU. */
};


/**
 * The status of both motors. Created by the OpenCR board after reading the
 * corresponding variables from the motors
 */
struct MotorStatus {
  uint16_t delta; /**< The time between samples, in &micro;s. */
  int16_t lCur;   /**< The current reported by the left motor, in 2.69 mA. */
  int16_t rCur;   /**< The current reported by the right motor, in 2.69 mA. */
  int32_t lVel;   /**< The velocity reported by the left motor, in 0.229 rev
                        / min. */
  int32_t rVel;   /**< The velocity reported by the right motor, in 0.229
                         rev / min. */
  int32_t lPos;   /**< The position reported by the left motor, in rev / 4096. */
  int32_t rPos;   /**< The position reported by the right motor, in rev / 4096. */
};


struct CurrentCommand {
  int16_t lCur; /**< The current desired on the left motor, in 2.69 mA. */
  int16_t rCur; /**< The current desired on the right motor, in 2.69 mA. */
};

struct VelocityCommand {
  int32_t lVel; /**< The velocity desired on the left motor, in 0.229 rev/min. */
  int32_t rVel; /**< The velocity desired on the right motor, in 0.229 rev/min. */
};

struct PositionCommand {
  int32_t lPos; /**< The position desired on the left motor, in rev / 4096. */
  int32_t rPos; /**< The position desired on the right motor, in rev / 4096. */
};

enum ControlModes : uint8_t {
  CUR_CTRL = 0,
  VEL_CTRL = 1,
  POS_CTRL = 4,
  NO_CTRL = 255
};

template<typename T>
const T& clamp(const T& v, const T& lo, const T& hi) {
  return (v < lo) ? lo : (hi < v) ? hi
                                  : v;
}

using std::array;


using DxlPort = dynamixel::PortHandler*;
using DxlHandler = dynamixel::PacketHandler*;
using DxlWriter = dynamixel::GroupBulkWrite;
using DxlReader = dynamixel::GroupBulkRead;

static constexpr int DESIRED_BAUDRATE = 2;

static constexpr array<int, 7> DXL_BAUDRATES = {9600, 57600, 115200, 1000000, 2000000, 3000000, 4500000};

DxlPort dxlPort;
DxlHandler dxlHandler;

constexpr size_t AdrressLen = 2;
constexpr size_t IndirectRegions = 2;
using address_t = uint16_t;
array<uint16_t, IndirectRegions> IndirectAddressStart = { 168, 578 };
array<uint16_t, IndirectRegions> IndirectAddressLen = { 28, 28 };
array<uint16_t, IndirectRegions> IndirectDataStart = { 224, 634 };

uint8_t read1Byte(uint8_t id, uint16_t address) {
  uint8_t dxl_error = 0;  // Dynamixel error
  uint8_t currentValue = 0;
  int dxl_comm_result = COMM_TX_FAIL;  // Communication result
  dxl_comm_result = dxlHandler->read1ByteTxRx(dxlPort, id, address, &currentValue, &dxl_error);
  return currentValue;
}

uint16_t read2Byte(uint8_t id, uint16_t address) {
  uint8_t dxl_error = 0;  // Dynamixel error
  uint16_t currentValue = -1;
  int dxl_comm_result = COMM_TX_FAIL;  // Communication result
  dxl_comm_result = dxlHandler->read2ByteTxRx(dxlPort, id, address, &currentValue, &dxl_error);
  return currentValue;
}

uint32_t read4Byte(uint8_t id, uint16_t address) {
  uint8_t dxl_error = 0;  // Dynamixel error
  uint32_t currentValue = -1;
  int dxl_comm_result = COMM_TX_FAIL;  // Communication result
  dxl_comm_result = dxlHandler->read4ByteTxRx(dxlPort, id, address, &currentValue, &dxl_error);
  return currentValue;
}


void ensure1Byte(uint8_t id, uint16_t address, uint8_t val) {
  uint8_t dxl_error = 0;  // Dynamixel error
  uint8_t currentValue = val + 1;
  int dxl_comm_result = COMM_TX_FAIL;  // Communication result
  dxl_comm_result = dxlHandler->read1ByteTxRx(dxlPort, id, address, &currentValue, &dxl_error);
  if (currentValue != val) {
    dxl_comm_result = dxlHandler->write1ByteTxRx(dxlPort, id, address, val, &dxl_error);
  }
}

void ensure2Byte(uint8_t id, uint16_t address, uint16_t val) {
  uint8_t dxl_error = 0;  // Dynamixel error
  uint16_t currentValue = val + 1;
  int dxl_comm_result = COMM_TX_FAIL;  // Communication result
  dxl_comm_result = dxlHandler->read2ByteTxRx(dxlPort, id, address, &currentValue, &dxl_error);
  if (currentValue != val) {
    dxl_comm_result = dxlHandler->write2ByteTxRx(dxlPort, id, address, val, &dxl_error);
  }
}

void ensure4Byte(uint8_t id, uint16_t address, uint32_t val) {
  uint8_t dxl_error = 0;  // Dynamixel error
  uint32_t currentValue = val + 1;
  int dxl_comm_result = COMM_TX_FAIL;  // Communication result
  dxl_comm_result = dxlHandler->read4ByteTxRx(dxlPort, id, address, &currentValue, &dxl_error);
  if (currentValue != val) {
    dxl_comm_result = dxlHandler->write4ByteTxRx(dxlPort, id, address, val, &dxl_error);
  }
}

bool setupIndirectData(uint8_t id, uint16_t target, uint16_t address, size_t len) {
  size_t region = 0;
  while (region < IndirectRegions) {
    if (IndirectDataStart[region] <= address && address + len <= IndirectDataStart[region] + IndirectAddressLen[region]) break;
  }
  if (region == IndirectRegions) return false;

  address_t curr_addr = IndirectAddressStart[region] + (address - IndirectDataStart[region]) * AdrressLen;
  for (size_t i = 0; i < len; i++) {
    ensure2Byte(id, curr_addr, target + i);
    curr_addr += AdrressLen;
  }
}


constexpr size_t IMU_BUF_SIZE = 20;
void WriteImuStatus(const ImuStatus& status) {
  uint8_t buf[IMU_BUF_SIZE + 1];
  buf[0] = IMU_STATUS;
  uint16_t writtenBytes = 1;
#define WRITE_MEM(mem) \
  memcpy(buf + writtenBytes, &status.mem, sizeof(status.mem)); \
  writtenBytes += sizeof(status.mem);
  WRITE_MEM(delta);
  WRITE_MEM(roll);
  WRITE_MEM(pitch);
  WRITE_MEM(yaw);
  WRITE_MEM(accX);
  WRITE_MEM(accY);
  WRITE_MEM(accZ);
#undef WRITE_MEM
  WriteSerialPackage(buf, writtenBytes);
}


constexpr size_t MOTOR_BUF_SIZE = 22;
void WriteMotorStatus(const MotorStatus& status) {
  uint8_t buf[MOTOR_BUF_SIZE + 1];
  buf[0] = MOTOR_STATUS;
  uint16_t writtenBytes = 1;
#define WRITE_MEM(mem) \
  memcpy(buf + writtenBytes, &status.mem, sizeof(status.mem)); \
  writtenBytes += sizeof(status.mem);
  WRITE_MEM(delta);
  WRITE_MEM(lCur);
  WRITE_MEM(rCur);
  WRITE_MEM(lVel);
  WRITE_MEM(rVel);
  WRITE_MEM(lPos);
  WRITE_MEM(rPos);
#undef WRITE_MEM
  WriteSerialPackage(buf, writtenBytes);
}

constexpr size_t MESSAGE_BUF_SIZE = 80;
void WriteMessage(const char* message) {
  uint8_t buf[MESSAGE_BUF_SIZE + 1];
  size_t len = strlen(message);
  size_t written = 0;
  buf[0] = OCR_MESSAGE;
  while (written < len) {
    size_t toWrite = min(len - written, size_t(MESSAGE_BUF_SIZE));
    memcpy(buf + 1, message + written, toWrite);
    digitalWrite(22, LOW);
    digitalWrite(23, HIGH);
    WriteSerialPackage(buf, toWrite + 1);
    digitalWrite(22, HIGH);
    digitalWrite(23, LOW);
    written += toWrite;
  }
}

constexpr const char* DEVICENAME = "OpenCR_DXL_Port";
constexpr float PROTOCOL_VERSION = 2.0;
constexpr uint8_t leftID = 1;
constexpr uint8_t rightID = 2;


DxlWriter* writer;
DxlReader* reader;


bool leftEnabled = 0;
bool rightEnabled = 0;

constexpr size_t CTRL_MODE_BUF_SIZE = 1;
ControlModes ctrl_mode;
void ReadCtrlModeCommand(const uint8_t* package) {
  ControlModes new_ctrl_mode;
  memcpy(&new_ctrl_mode, package, sizeof(ControlModes));
  switch(new_ctrl_mode) {
    case ControlModes::CUR_CTRL:
    case ControlModes::VEL_CTRL:
    case ControlModes::POS_CTRL:
      if(ctrl_mode != new_ctrl_mode) {

        // Disable motors

        dxlHandler->write1ByteTxOnly(dxlPort, leftID, 64, 0);
        dxlHandler->write1ByteTxOnly(dxlPort, rightID, 64, 0);
        

        ctrl_mode = new_ctrl_mode;
        uint8_t dxl_error = 0;  // Dynamixel error
        dxlHandler->write1ByteTxOnly(dxlPort, leftID, 11, ctrl_mode);
        dxlHandler->write1ByteTxOnly(dxlPort, rightID, 11, ctrl_mode);

        // Re enable motors

        dxlHandler->write1ByteTxOnly(dxlPort, leftID, 64, 1);
        dxlHandler->write1ByteTxOnly(dxlPort, rightID, 64, 1);
      }
  }
}

constexpr size_t CUR_BUF_SIZE = 4;
CurrentCommand lastCurCommand;
uint32_t lastCurCommandTime;
bool hasLastCurCommand = false;
void ReadCurCommand(const uint8_t* package) {
  size_t readBytes = 0;
#define READ_MEM(mem) \
  memcpy(&lastCurCommand.mem, package + readBytes, sizeof(lastCurCommand.mem)); \
  readBytes += sizeof(lastCurCommand.mem);
  READ_MEM(lCur);
  READ_MEM(rCur);
#undef READ_MEM
  lastCurCommandTime = micros();
  hasLastCurCommand = true;
}

constexpr size_t VEL_BUF_SIZE = 8;
VelocityCommand lastVelCommand;
uint32_t lastVelCommandTime;
bool hasLastVelCommand = false;
void ReadVelCommand(const uint8_t* package) {
  size_t readBytes = 0;
#define READ_MEM(mem) \
  memcpy(&lastVelCommand.mem, package + readBytes, sizeof(lastVelCommand.mem)); \
  readBytes += sizeof(lastVelCommand.mem);
  READ_MEM(lVel);
  READ_MEM(rVel);
#undef READ_MEM
  lastVelCommandTime = micros();
  hasLastVelCommand = true;
}

constexpr size_t POS_BUF_SIZE = 8;
PositionCommand lastPosCommand;
uint32_t lastPosCommandTime;
bool hasLastPosCommand = false;
void ReadPosCommand(const uint8_t* package) {
  size_t readBytes = 0;
#define READ_MEM(mem) \
  memcpy(&lastPosCommand.mem, package + readBytes, sizeof(lastPosCommand.mem)); \
  readBytes += sizeof(lastPosCommand.mem);
  READ_MEM(lPos);
  READ_MEM(rPos);
#undef READ_MEM
  lastPosCommandTime = micros();
  hasLastPosCommand = true;
}

void ReadMessage(const char* message, size_t length) {
  WriteMessage(message);
}


void packageCall(uint8_t* data, uint32_t size, void* u_data) {
  if (size) {
    switch (*data) {
      case MessageTypes::OCR_MESSAGE:
        ReadMessage((char*)data + 1, size - 1);
        return;
      case MessageTypes::CTRL_MODE:
        ReadCtrlModeCommand(data + 1);
        return;
      case MessageTypes::CUR_COMMAND:
        ReadCurCommand(data + 1);
        return;
      case MessageTypes::VEL_COMMAND:
        ReadVelCommand(data + 1);
        return;
      case MessageTypes::POS_COMMAND:
        ReadPosCommand(data + 1);
        return;
    }
  }
}

uint32_t availToWriteCall(void* u_data) {
  if(!Serial) return 0;
  return 256; // Tamaño simulado del buffer para permitir la escritura
}

uint32_t writeCall(uint8_t* data, uint32_t size, void* u_data) {
  if(!Serial) return 0;
  return Serial.write(data, size);
}

uint32_t availToReadCall(void* u_data) {
  if(!Serial) return 0;
  return Serial.available();
}

uint32_t readCall(uint8_t* data, uint32_t size, void* u_data) {
  if(!Serial) return 0;
  int i = 0;
  while (Serial.available() && i < size) {
    data[i] = Serial.read();
    ++i;
  }
  return i;
}

void setupSerial() {
  setPackageUserData(&Serial);
  setPackageCallback(packageCall);

  setAvailToWriteUserData(&Serial);
  setAvailToWriteCallback(availToWriteCall);

  setWriteUserData(&Serial);
  setWriteCallback(writeCall);

  setAvailToReadUserData(&Serial);
  setAvailToReadCallback(availToReadCall);

  setReadUserData(&Serial);
  setReadCallback(readCall);

  Serial.begin(1152000);
  //while (!Serial) {}
}

int32_t lastIMUTime;
int32_t lastMotorTime;

void setupIMU() {
  IMU.begin(1111);
  IMU.SEN.acc_cali_start();
  while (!IMU.SEN.acc_cali_get_done()) {
    IMU.update();
  }
  lastIMUTime = micros();
}



constexpr address_t GoalCurAddr = 102;
constexpr address_t GoalCurLen = 2;
constexpr address_t GoalVelAddr = 104;
constexpr address_t GoalVelLen = 4;
constexpr address_t GoalPosAddr = 116;
constexpr address_t GoalPosLen = 4;
constexpr address_t PresCurAddr = 126;
constexpr address_t PresCurLen = 2;
constexpr address_t PresVelAddr = 128;
constexpr address_t PresVelLen = 4;
constexpr address_t PresPosAddr = 132;
constexpr address_t PresPosLen = 4;
constexpr address_t EnabledAddr = 64;
constexpr address_t EnabledLen = 1;

struct IndirectStatus {
  int32_t position;
  int32_t velocity;
  int16_t current;
  int8_t enabled;
};
static_assert(offsetof(IndirectStatus, position) == 0, "wrong offset");
static_assert(offsetof(IndirectStatus, velocity) == 4, "wrong offset");
static_assert(offsetof(IndirectStatus, current) == 8, "wrong offset");
static_assert(offsetof(IndirectStatus, enabled) == 10, "wrong offset");



bool setupMotors() {
  dxlPort = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dxlHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  writer = new DxlWriter(dxlPort, dxlHandler);
  reader = new DxlReader(dxlPort, dxlHandler);

   if (dxlPort->openPort()) {
  } else {
    //Handle error?
    return false;
  }

  for(int i = 0; i < DXL_BAUDRATES.size(); i++) {
    dxlPort->setBaudRate(DXL_BAUDRATES[i]);
    uint16_t model_number;
    int dxl_comm_result;
    dxl_comm_result = dxlHandler->ping(dxlPort, leftID, &model_number);
    if (dxl_comm_result == COMM_SUCCESS) {
      ensure1Byte(leftID, 8, DESIRED_BAUDRATE);
    }
    dxl_comm_result = dxlHandler->ping(dxlPort, rightID, &model_number);
    if (dxl_comm_result == COMM_SUCCESS) {
      ensure1Byte(rightID, 8, DESIRED_BAUDRATE);
    }
  }

  if (dxlPort->setBaudRate(DXL_BAUDRATES[DESIRED_BAUDRATE])) {
  } else {
    //Handle error?
    return false;
  }

  setupIndirectData(leftID, PresPosAddr, IndirectDataStart[0] + offsetof(IndirectStatus, position), PresPosLen);
  setupIndirectData(rightID, PresPosAddr, IndirectDataStart[0] + offsetof(IndirectStatus, position), PresPosLen);
  setupIndirectData(leftID, PresVelAddr, IndirectDataStart[0] + offsetof(IndirectStatus, velocity), PresVelLen);
  setupIndirectData(rightID, PresVelAddr, IndirectDataStart[0] + offsetof(IndirectStatus, velocity), PresVelLen);
  setupIndirectData(leftID, PresCurAddr, IndirectDataStart[0] + offsetof(IndirectStatus, current), PresCurLen);
  setupIndirectData(rightID, PresCurAddr, IndirectDataStart[0] + offsetof(IndirectStatus, current), PresCurLen);
  setupIndirectData(leftID, EnabledAddr, IndirectDataStart[0] + offsetof(IndirectStatus, enabled), EnabledLen);
  setupIndirectData(rightID, EnabledAddr, IndirectDataStart[0] + offsetof(IndirectStatus, enabled), EnabledLen);

  reader->addParam(leftID, IndirectDataStart[0], 11);
  reader->addParam(rightID, IndirectDataStart[0], 11);

  //Minimum return delay
  ensure1Byte(leftID, 9, 0);
  ensure1Byte(rightID, 9, 0);
  ensure4Byte(leftID, 44, 1023);
  ensure4Byte(rightID, 44, 1023);
  ensure1Byte(leftID, 10, 0);
  ensure1Byte(rightID, 10, 1);

  ctrl_mode = NO_CTRL;

  reader->txPacket();

  lastMotorTime = micros();
  return true;
}

void setup() {
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  setupSerial();
  setupIMU();
  setupMotors();
}
ImuStatus imuStatus;
MotorStatus motorStatus;

int16_t imuDeltaCorrection = 0;
void pollIMU() {
  int32_t now = micros();
  auto delta = now - lastIMUTime;
  uint16_t imuDelta;
  if (delta + imuDeltaCorrection >= 1000 && (imuDelta = IMU.update()) > 0) {
    imuDeltaCorrection = clamp<int32_t>(imuDeltaCorrection + imuDelta - 1000, -100, 100);
    imuStatus.delta = imuDelta;
    lastIMUTime = now;
    imuStatus.roll = IMU.rpy[0];
    imuStatus.pitch = IMU.rpy[1];
    imuStatus.yaw = IMU.rpy[2];
    imuStatus.accX = IMU.accData[0];
    imuStatus.accY = IMU.accData[1];
    imuStatus.accZ = IMU.accData[2];
    WriteImuStatus(imuStatus);
  }
}

int16_t motorDeltaCorrection = 0;
const int32_t maxCommandTime = 10000;
const int32_t timeCommandFull = 1500;

void pollMotors() {

  uint32_t now = micros();
  if(ctrl_mode == ControlModes::CUR_CTRL) {
    int16_t leftGoal = 0;
    int16_t rightGoal = 0;

    if (now - lastCurCommandTime > maxCommandTime) {
      hasLastCurCommand = false;
    }
    if (hasLastCurCommand) {
      leftGoal = lastCurCommand.lCur;
      rightGoal = lastCurCommand.rCur;
      if (now - lastCurCommandTime > timeCommandFull) {

        float factor = (maxCommandTime - (now - lastCurCommandTime)) / float(maxCommandTime - timeCommandFull);

        leftGoal *= factor;
        rightGoal *= factor;
      }
    }
    writer->addParam(leftID, GoalCurAddr, GoalCurLen, (uint8_t*)&leftGoal);
    writer->addParam(rightID, GoalCurAddr, GoalCurLen, (uint8_t*)&rightGoal);
  } else if (ctrl_mode == ControlModes::VEL_CTRL) {
    int32_t leftGoal = 0;
    int32_t rightGoal = 0;

    if (now - lastVelCommandTime > maxCommandTime) {
      hasLastVelCommand = false;
    }
    if (hasLastVelCommand) {
      leftGoal = lastVelCommand.lVel;
      rightGoal = lastVelCommand.rVel;
      if (now - lastVelCommandTime > timeCommandFull) {

        float factor = (maxCommandTime - (now - lastVelCommandTime)) / float(maxCommandTime - timeCommandFull);
        leftGoal *= factor;
        rightGoal *= factor;
      }
    }
    writer->addParam(leftID, GoalVelAddr, GoalVelLen, (uint8_t*)&leftGoal);
    writer->addParam(rightID, GoalVelAddr, GoalVelLen, (uint8_t*)&rightGoal);
  } else if (ctrl_mode == ControlModes::POS_CTRL) {
    int32_t leftGoal = motorStatus.lPos;
    int32_t rightGoal = motorStatus.rPos;

    if (now - lastPosCommandTime > maxCommandTime) {
      hasLastPosCommand = false;
    }
    if (hasLastPosCommand) {
      leftGoal = lastPosCommand.lPos;
      rightGoal = lastPosCommand.rPos;
      if (now - lastPosCommandTime > timeCommandFull) {

        float factor = (maxCommandTime - (now - lastPosCommandTime)) / float(maxCommandTime - timeCommandFull);
        leftGoal = (leftGoal - motorStatus.lPos) * factor;
        leftGoal += motorStatus.lPos;
        rightGoal = (rightGoal - motorStatus.rPos) * factor;
        rightGoal += motorStatus.rPos;
      }
    }
    writer->addParam(leftID, GoalPosAddr, GoalPosLen, (uint8_t*)&leftGoal);
    writer->addParam(rightID, GoalPosAddr, GoalPosLen, (uint8_t*)&rightGoal);
  }


  auto delta = now - lastMotorTime;
  if (delta + motorDeltaCorrection >= 1000 && reader->rxPacket() != COMM_NOT_AVAILABLE) {
    motorDeltaCorrection = clamp<int32_t>(motorDeltaCorrection + delta - 1000, -100, 100);
    motorStatus.delta = delta;
    lastMotorTime = now;
    motorStatus.lPos = reader->getData(leftID, IndirectDataStart[0] + offsetof(IndirectStatus, position), PresPosLen);
    motorStatus.rPos = reader->getData(rightID, IndirectDataStart[0] + offsetof(IndirectStatus, position), PresPosLen);
    motorStatus.lVel = reader->getData(leftID, IndirectDataStart[0] + offsetof(IndirectStatus, velocity), PresVelLen);
    motorStatus.rVel = reader->getData(rightID, IndirectDataStart[0] + offsetof(IndirectStatus, velocity), PresVelLen);
    motorStatus.lCur = reader->getData(leftID, IndirectDataStart[0] + offsetof(IndirectStatus, current), PresCurLen);
    motorStatus.rCur = reader->getData(rightID, IndirectDataStart[0] + offsetof(IndirectStatus, current), PresCurLen);
    leftEnabled = reader->getData(leftID, IndirectDataStart[0] + offsetof(IndirectStatus, enabled), EnabledLen);
    rightEnabled = reader->getData(rightID, IndirectDataStart[0] + offsetof(IndirectStatus, enabled), EnabledLen);
    if(ctrl_mode != NO_CTRL) {
      writer->txPacket();
      writer->clearParam();
    }
    WriteMotorStatus(motorStatus);
    reader->txPacket();

    if(leftEnabled || rightEnabled) {
      WriteMessage("Motors Enabled");
    } else {
      WriteMessage("Motors Disabled");
    }
  }
}

void loop() {
  pollSerial();
  pollIMU();
  pollMotors();
}
