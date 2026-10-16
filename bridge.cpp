// bridge.cpp -- Puente Raspberry Pi <-> OpenCR para control por torque.
//
//   Laptop (Python: ZNN + modelo dinámico)
//        |  SSH: stdin/stdout del proceso, tramas binarias con CRC16
//        v
//   bridge (este programa, lazo a 1 kHz)  --USB serial-->  OpenCR  -->  XM430
//
// Orden de los arreglos en el protocolo con la laptop: indice 0 = rueda
// DERECHA, indice 1 = rueda IZQUIERDA (igual que eta = [phi_R, phi_L] del informe).
//
// Por cada rueda j, a 1 kHz:
//   eta_c(t) = eta_c0 + deta_c0 * (t - t_llegada)      (extrapolación acotada)
//   tau_j    = tau_ff_j + kd * (eta_c_j - eta_j)       (feedforward + realim. local)
//   i_j      = tau_j / (ktau * 2.69 mA)  -> entero, saturado a +-imax
//
// Seguridad: si la referencia tiene mas de --wd-ms de antiguedad, enable=0 o la
// sesion SSH muere (EOF en stdin), el puente frena con amortiguamiento
// tau = -kd_brake * eta y termina.
//
// Compilar: ver CMakeLists.txt (necesita C++20 por motor_status.cpp).

#define SERIAL_MESSAGE_IMPL
#include "SerialMessageObj.h"
#include "include/current_command.h"
#include "include/imu_status.h"
#include "include/motor_status.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <numbers>
#include <string>
#include <thread>
#include <vector>

#include <errno.h>
#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

using Clock = std::chrono::steady_clock;
constexpr int W_R = 0; // indice rueda derecha en el protocolo
constexpr int W_L = 1; // indice rueda izquierda

// ------------------------------------------------------------------ tramas
#pragma pack(push, 1)
struct RefFrame { // laptop -> Pi, 38 bytes
	uint16_t sync;      // 0x5AA5
	uint8_t type;       // 1
	uint8_t flags;      // bit0 = enable
	uint32_t seq;
	float eta_c[2];     // rad/s   [R, L]
	float deta_c[2];    // rad/s^2 [R, L]
	float tau_ff[2];    // N*m     [R, L]
	float kd;           // N*m / (rad/s)
	uint16_t crc;
};
struct TelFrame { // Pi -> laptop, 62 bytes
	uint16_t sync;
	uint8_t type;       // 2
	uint8_t flags;      // bit0 = referencia fresca
	uint32_t seq_echo;  // ultima seq de referencia aceptada
	uint32_t t_ms;      // ms desde el arranque del puente
	float eta[2];       // rad/s medidas [R, L]
	float phi[2];       // rad medidas   [R, L]
	float cur_a[2];     // A medidas     [R, L]
	int16_t cmd_units[2]; // corriente ordenada, unidades de 2.69 mA [R, L]
	float yaw_deg;      // yaw del IMU del OpenCR
	float odo[3];       // x, y, theta por odometria de ruedas (m, m, rad)
	float age_ms;       // antiguedad de la referencia al enviar esta trama
	uint16_t crc;
};
#pragma pack(pop)
static_assert(sizeof(RefFrame) == 38, "RefFrame size");
static_assert(sizeof(TelFrame) == 62, "TelFrame size");
constexpr uint16_t SYNC = 0x5AA5;

uint16_t crc16(const uint8_t* d, size_t n) { // CRC-16/CCITT-FALSE
	uint16_t c = 0xFFFF;
	for (size_t i = 0; i < n; i++) {
		c ^= uint16_t(d[i]) << 8;
		for (int b = 0; b < 8; b++)
			c = (c & 0x8000) ? uint16_t((c << 1) ^ 0x1021) : uint16_t(c << 1);
	}
	return c;
}

struct RefParser {
	std::vector<uint8_t> buf;
	template <typename F> void feed(const uint8_t* p, size_t n, F&& onFrame) {
		buf.insert(buf.end(), p, p + n);
		size_t i = 0;
		while (buf.size() - i >= sizeof(RefFrame)) {
			if (buf[i] == 0xA5 && buf[i + 1] == 0x5A) {
				RefFrame f;
				memcpy(&f, &buf[i], sizeof f);
				if (f.type == 1 && crc16(&buf[i], sizeof f - 2) == f.crc) {
					onFrame(f);
					i += sizeof f;
					continue;
				}
			}
			++i;
		}
		buf.erase(buf.begin(), buf.begin() + i);
		if (buf.size() > 4096) buf.clear();
	}
};

// ------------------------------------------------------------------ config
struct Config {
	std::string dev   = "/dev/ttyACM0";
	float ktau        = 1.05f;   // N*m/A (el mismo que motor_status.cpp)
	int imax          = 100;     // unidades de 2.69 mA (100 -> 0.27 A)
	float wd_ms       = 100.f;   // watchdog de referencia
	float extrap_ms   = 50.f;    // maxima extrapolacion de eta_c
	float kd_brake    = 0.05f;   // amortiguamiento al frenar
	float tel_hz      = 50.f;
	float R           = 0.033f;  // radio de rueda
	float L           = 0.1435f; // semi-separacion entre ruedas
	int sign_l        = 1;       // +-1: corrige sentido de giro de cada motor
	int sign_r        = 1;
	bool sim          = false;
	// planta simulada: m_eff, J_eff, mu, d, c
	float s_m = 3.0f, s_J = 0.06f, s_mu = 0.0f, s_d = 0.02f, s_c = 0.03f;
	float s_ktau      = -1.f;    // si <0 usa ktau
};

static void usage() {
	fprintf(stderr,
	        "bridge [--dev PATH] [--ktau N] [--imax U] [--wd-ms MS] [--extrap-ms MS]\n"
	        "       [--kd-brake K] [--tel-hz HZ] [--R M] [--L M] [--sign-l +-1] [--sign-r +-1]\n"
	        "       [--sim] [--sim-params m_eff,J_eff,mu,d,c] [--sim-ktau N]\n");
}

static bool parseArgs(int argc, char** argv, Config& c) {
	for (int i = 1; i < argc; i++) {
		std::string a = argv[i];
		auto next = [&]() -> const char* {
			if (i + 1 >= argc) { usage(); exit(2); }
			return argv[++i];
		};
		if (a == "--dev") c.dev = next();
		else if (a == "--ktau") c.ktau = atof(next());
		else if (a == "--imax") c.imax = atoi(next());
		else if (a == "--wd-ms") c.wd_ms = atof(next());
		else if (a == "--extrap-ms") c.extrap_ms = atof(next());
		else if (a == "--kd-brake") c.kd_brake = atof(next());
		else if (a == "--tel-hz") c.tel_hz = atof(next());
		else if (a == "--R") c.R = atof(next());
		else if (a == "--L") c.L = atof(next());
		else if (a == "--sign-l") c.sign_l = atoi(next()) >= 0 ? 1 : -1;
		else if (a == "--sign-r") c.sign_r = atoi(next()) >= 0 ? 1 : -1;
		else if (a == "--sim") c.sim = true;
		else if (a == "--sim-ktau") c.s_ktau = atof(next());
		else if (a == "--sim-params") {
			if (sscanf(next(), "%f,%f,%f,%f,%f", &c.s_m, &c.s_J, &c.s_mu, &c.s_d, &c.s_c) != 5) {
				usage(); return false;
			}
		} else { usage(); return false; }
	}
	if (c.s_ktau < 0) c.s_ktau = c.ktau;
	return true;
}

// ------------------------------------------------------------------ OpenCR
enum MsgType : uint8_t { OCR_MESSAGE = 0, IMU_STATUS_T, MOTOR_STATUS_T, CTRL_MODE, CUR_COMMAND };
constexpr uint8_t CUR_CTRL = 0;

struct OpenCR {
	SerialMessage sm;
	int fd = -1;
	MotorStatus motor;
	ImuStatus imu{};

	static uint32_t availToWrite(void* u) {
		int res = 0;
		ioctl(*(int*)u, TIOCOUTQ, &res);
		return res ? 0 : 256;
	}
	static uint32_t writeCb(uint8_t* d, uint32_t n, void* u) {
		auto r = write(*(int*)u, d, n);
		return r < 0 ? 0 : uint32_t(r);
	}
	static uint32_t availToRead(void* u) {
		int res = 0;
		ioctl(*(int*)u, FIONREAD, &res);
		return res;
	}
	static uint32_t readCb(uint8_t* d, uint32_t n, void* u) {
		auto r = read(*(int*)u, d, n);
		return r < 0 ? 0 : uint32_t(r);
	}
	static void packageCb(uint8_t* data, uint32_t size, void* u) {
		auto& self = *(OpenCR*)u;
		if (!size) return;
		const uint8_t* p = data + 1;
		size_t o = 0;
		switch (*data) {
		case OCR_MESSAGE: break; // texto de depuracion del firmware: se ignora
		case IMU_STATUS_T:
			if (size == 21) {
#define RD(m) memcpy(&self.imu.m, p + o, sizeof(self.imu.m)); o += sizeof(self.imu.m);
				RD(delta) RD(roll) RD(pitch) RD(yaw) RD(accX) RD(accY) RD(accZ)
#undef RD
			}
			break;
		case MOTOR_STATUS_T:
			if (size == 23) {
#define RD(m) memcpy(&self.motor.m, p + o, sizeof(self.motor.m)); o += sizeof(self.motor.m);
				RD(delta) RD(lCur) RD(rCur) RD(lVel) RD(rVel) RD(lPos) RD(rPos)
#undef RD
			}
			break;
		}
	}

	bool open(const std::string& path, double timeout_s) {
		auto t0 = Clock::now();
		while ((fd = ::open(path.c_str(), O_RDWR | O_NOCTTY)) < 0) {
			if (std::chrono::duration<double>(Clock::now() - t0).count() > timeout_s) {
				fprintf(stderr, "[bridge] no se pudo abrir %s: %s\n", path.c_str(), strerror(errno));
				return false;
			}
			fprintf(stderr, "[bridge] esperando %s ...\n", path.c_str());
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
		termios tty;
		if (tcgetattr(fd, &tty) != 0) return false;
		tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
		tty.c_cflag |= CS8 | CREAD | CLOCAL;
		tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
		tty.c_iflag &= ~(IXON | IXOFF | IXANY | INPCK | IUCLC | IMAXBEL | INLCR | IGNCR | ICRNL);
		tty.c_oflag &= ~(OPOST | ONLCR);
		tty.c_cc[VTIME] = 0;
		tty.c_cc[VMIN] = 0;
		cfsetispeed(&tty, B1152000);
		cfsetospeed(&tty, B1152000);
		if (tcsetattr(fd, TCSANOW, &tty) != 0) return false;
		serial_struct ss;
		if (ioctl(fd, TIOCGSERIAL, &ss) == 0) {
			ss.flags |= ASYNC_LOW_LATENCY;
			ioctl(fd, TIOCSSERIAL, &ss);
		}
		tcflush(fd, TCIOFLUSH);
		sm.setPackageUserData(this);
		sm.setPackageCallback(packageCb);
		sm.setAvailToWriteUserData(&fd);
		sm.setAvailToWriteCallback(availToWrite);
		sm.setWriteUserData(&fd);
		sm.setWriteCallback(writeCb);
		sm.setAvailToReadUserData(&fd);
		sm.setAvailToReadCallback(availToRead);
		sm.setReadUserData(&fd);
		sm.setReadCallback(readCb);
		return true;
	}
	void sendCtrlMode(uint8_t mode) {
		uint8_t b[2] = {CTRL_MODE, mode};
		sm.writePackage(b, 2);
	}
	void sendCurrent(int16_t l, int16_t r) {
		uint8_t b[5] = {CUR_COMMAND};
		memcpy(b + 1, &l, 2);
		memcpy(b + 3, &r, 2);
		sm.writePackage(b, 5);
	}
};

// ------------------------------------------------- planta simulada (--sim)
// Reproduce  M eta' + V(eta) eta + F(eta) = tau  con (m_eff, J_eff, mu) y friccion,
// y entrega MotorStatus con la misma cuantizacion que el hardware real.
constexpr float kVelUnit = 0.229f * 2.f * std::numbers::pi_v<float> / 60.f; // rad/s por unidad
constexpr float kPosUnit = 2.f * std::numbers::pi_v<float> / 4096.f;       // rad por unidad
constexpr float kCurUnit = 2.69e-3f;                                        // A por unidad

struct SimPlant {
	double eta[2] = {0, 0}; // [R, L]
	double phi[2] = {0, 0};
	double p1 = 0, p2 = 0, p3 = 0, d = 0, c = 0, eps = 0.1, R = 0, L = 1, ktau = 1;
	void init(const Config& cf) {
		R = cf.R; L = cf.L; ktau = cf.s_ktau; d = cf.s_d; c = cf.s_c;
		p1 = R * R * (cf.s_m / 4 + cf.s_J / (4 * L * L));
		p2 = R * R * (cf.s_m / 4 - cf.s_J / (4 * L * L));
		p3 = R * R * cf.s_mu / (2 * L);
	}
	void step(double dt, int16_t uR, int16_t uL) {
		double tau[2] = {uR * kCurUnit * ktau, uL * kCurUnit * ktau};
		double w = R * (eta[0] - eta[1]) / (2 * L);
		double rhs[2] = {tau[0] - p3 * w * eta[1] - (d * eta[0] + c * std::tanh(eta[0] / eps)),
		                 tau[1] + p3 * w * eta[0] - (d * eta[1] + c * std::tanh(eta[1] / eps))};
		double det = p1 * p1 - p2 * p2;
		double a0 = (p1 * rhs[0] - p2 * rhs[1]) / det;
		double a1 = (-p2 * rhs[0] + p1 * rhs[1]) / det;
		eta[0] += dt * a0;
		eta[1] += dt * a1;
		phi[0] += dt * eta[0];
		phi[1] += dt * eta[1];
	}
	void fill(MotorStatus& m, ImuStatus& imu, int16_t uR, int16_t uL) const {
		m.delta = 1000;
		m.rCur = uR; m.lCur = uL;
		m.rVel = int32_t(std::lround(eta[0] / kVelUnit));
		m.lVel = int32_t(std::lround(eta[1] / kVelUnit));
		m.rPos = int32_t(std::lround(phi[0] / kPosUnit));
		m.lPos = int32_t(std::lround(phi[1] / kPosUnit));
		imu.yaw = float(R * (phi[0] - phi[1]) / (2 * L) * 180.0 / std::numbers::pi);
	}
};

// ------------------------------------------------------------- odometria
struct Odometry {
	double x = 0, y = 0, th = 0, pR = 0, pL = 0;
	bool init = false;
	void update(double phiR, double phiL, double R, double L) {
		if (!init) { pR = phiR; pL = phiL; init = true; return; }
		double dR = phiR - pR, dL = phiL - pL;
		pR = phiR; pL = phiL;
		double ds = R * (dR + dL) / 2, dth = R * (dR - dL) / (2 * L);
		x += ds * std::cos(th + dth / 2);
		y += ds * std::sin(th + dth / 2);
		th += dth;
	}
};

static volatile sig_atomic_t g_stop = 0;
static void onSignal(int) { g_stop = 1; }

int main(int argc, char** argv) {
	Config cfg;
	if (!parseArgs(argc, argv, cfg)) return 2;
	signal(SIGPIPE, SIG_IGN);
	signal(SIGINT, onSignal);
	signal(SIGTERM, onSignal);
	signal(SIGHUP, onSignal);
	fcntl(0, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK);
	fcntl(1, F_SETFL, fcntl(1, F_GETFL) | O_NONBLOCK);

	OpenCR ocr;
	SimPlant sim;
	if (cfg.sim) {
		sim.init(cfg);
		fprintf(stderr, "[bridge] MODO SIMULACION (sin OpenCR)\n");
	} else if (!ocr.open(cfg.dev, 30.0)) {
		return 1;
	}
	fprintf(stderr, "[bridge] listo. ktau=%.3f N*m/A imax=%d wd=%.0f ms\n", cfg.ktau, cfg.imax, cfg.wd_ms);

	RefFrame ref{};
	bool haveRef = false;
	uint32_t lastSeq = 0;
	Clock::time_point refArrival = Clock::now();
	RefParser parser;
	Odometry odo;

	const auto t0 = Clock::now();
	auto nextTick = t0;
	auto nextTel = t0;
	auto nextMode = t0;
	auto nextLog = t0 + std::chrono::seconds(1);
	const auto period = std::chrono::microseconds(1000);
	const auto telPeriod = std::chrono::microseconds(int(1e6f / cfg.tel_hz));
	int16_t cmdR = 0, cmdL = 0;
	bool eof = false;
	Clock::time_point eofAt;
	float tau_log[2] = {0, 0};

	while (!g_stop) {
		if (!cfg.sim) ocr.sm.poll();

		uint8_t rbuf[512];
		ssize_t n = read(0, rbuf, sizeof rbuf);
		if (n > 0) {
			parser.feed(rbuf, size_t(n), [&](const RefFrame& f) {
				if (!haveRef || int32_t(f.seq - lastSeq) > 0) {
					ref = f; lastSeq = f.seq; haveRef = true;
					refArrival = Clock::now();
				}
			});
		} else if (n == 0 && !eof) {
			eof = true;
			eofAt = Clock::now();
			fprintf(stderr, "[bridge] EOF en stdin (sesion SSH cerrada): frenando.\n");
		}
		if (eof && Clock::now() - eofAt > std::chrono::milliseconds(400)) break;

		auto now = Clock::now();
		if (now < nextTick) {
			std::this_thread::sleep_for(std::chrono::microseconds(50));
			continue;
		}
		nextTick += period;
		if (now - nextTick > std::chrono::milliseconds(50)) nextTick = now + period;

		// ---- estado medido
		if (cfg.sim) {
			sim.step(1e-3, cmdR, cmdL);
			sim.fill(ocr.motor, ocr.imu, cmdR, cmdL);
		}
		if (ocr.motor.delta == 0) { // aun no llega telemetria del OpenCR
			if (!cfg.sim) ocr.sendCurrent(0, 0);
			continue;
		}
		auto vel = ocr.motor.getVelocity(); // {izq, der} rad/s
		auto pos = ocr.motor.getPosition();
		auto cur = ocr.motor.getCurrent();
		float eta[2] = {cfg.sign_r * vel[1], cfg.sign_l * vel[0]};
		float phi[2] = {cfg.sign_r * pos[1], cfg.sign_l * pos[0]};
		odo.update(phi[W_R], phi[W_L], cfg.R, cfg.L);

		// ---- ley de torque
		float ageMs = haveRef ? std::chrono::duration<float, std::milli>(now - refArrival).count() : 1e9f;
		bool fresh = haveRef && !eof && (ref.flags & 1) && ageMs < cfg.wd_ms;
		float tau[2];
		for (int j = 0; j < 2; j++) {
			if (fresh) {
				float ae = std::min(ageMs, cfg.extrap_ms) * 1e-3f;
				float etac = ref.eta_c[j] + ref.deta_c[j] * ae;
				tau[j] = ref.tau_ff[j] + ref.kd * (etac - eta[j]);
			} else {
				tau[j] = -cfg.kd_brake * eta[j];
			}
			tau_log[j] = tau[j];
		}
		const float perUnit = cfg.ktau * kCurUnit;
		int16_t u[2];
		for (int j = 0; j < 2; j++) {
			long k = std::lround(tau[j] / perUnit);
			u[j] = int16_t(std::clamp<long>(k, -cfg.imax, cfg.imax));
		}
		cmdR = u[W_R];
		cmdL = u[W_L];
		if (!cfg.sim) {
			if (now >= nextMode) { // idempotente: el firmware ignora si ya esta en ese modo
				ocr.sendCtrlMode(CUR_CTRL);
				nextMode = now + std::chrono::seconds(1);
			}
			ocr.sendCurrent(int16_t(cfg.sign_l * cmdL), int16_t(cfg.sign_r * cmdR));
		}

		// ---- telemetria
		if (now >= nextTel) {
			nextTel = now + telPeriod;
			TelFrame t{};
			t.sync = SYNC; t.type = 2; t.flags = fresh ? 1 : 0;
			t.seq_echo = lastSeq;
			t.t_ms = uint32_t(std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count());
			t.eta[0] = eta[W_R]; t.eta[1] = eta[W_L];
			t.phi[0] = phi[W_R]; t.phi[1] = phi[W_L];
			t.cur_a[0] = cfg.sign_r * cur[1]; t.cur_a[1] = cfg.sign_l * cur[0];
			t.cmd_units[0] = cmdR; t.cmd_units[1] = cmdL;
			t.yaw_deg = ocr.imu.yaw;
			t.odo[0] = float(odo.x); t.odo[1] = float(odo.y); t.odo[2] = float(odo.th);
			t.age_ms = std::min(ageMs, 1e6f);
			t.crc = crc16(reinterpret_cast<uint8_t*>(&t), sizeof t - 2);
			ssize_t w = write(1, &t, sizeof t); // no bloqueante: si falla se descarta
			(void)w;
		}
		if (now >= nextLog) {
			nextLog = now + std::chrono::seconds(1);
			fprintf(stderr, "[bridge] fresh=%d age=%.0fms eta=[%.2f %.2f] tau=[%.3f %.3f] cmd=[%d %d] motor_dt=%uus\n",
			        int(fresh), std::min(ageMs, 9999.f), eta[0], eta[1], tau_log[0], tau_log[1], cmdR, cmdL,
			        unsigned(ocr.motor.delta));
		}
	}
	// parada: corriente cero
	if (!cfg.sim && ocr.fd >= 0) {
		for (int i = 0; i < 20; i++) {
			ocr.sendCurrent(0, 0);
			ocr.sm.poll();
			std::this_thread::sleep_for(std::chrono::milliseconds(2));
		}
	}
	fprintf(stderr, "[bridge] terminado.\n");
	return 0;
}
