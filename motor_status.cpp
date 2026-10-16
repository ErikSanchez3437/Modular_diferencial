#include <chrono>
#include <cstdint>
#include <include/motor_status.h>
#include <numbers>
#include <ratio>
#include <type_traits>

using std::false_type;
using std::micro;
using std::milli;
using std::ratio;
using std::true_type;
using std::numbers::pi_v;

template <typename T> struct is_ratio : false_type {};

template <intmax_t Num, intmax_t Den>
struct is_ratio<ratio<Num, Den>> : true_type {};

template <typename T> constexpr bool is_ratio_v = is_ratio<T>::value;

template <typename T>
concept Ratio = is_ratio_v<T>;

template <typename T, Ratio R>
constexpr T as = T(R::type::num) / T(R::type::den);

static constexpr float currentFactor = 2.69f * as<float, milli>;
static constexpr float rpmToRadps =
    2.0f * pi_v<float> / as<float, std::chrono::minutes::period>;
static constexpr float velocityFactor = 0.229 * rpmToRadps;
using pulseRatio = ratio<1, 4096>;
static constexpr float positionFactor = 2 * pi_v<float> * as<float, pulseRatio>;

MotorStatus::MotorStatus()
    : delta{}, lCur{}, rCur{}, lVel{}, rVel{}, lPos{}, rPos{} {};

float MotorStatus::getDelta() { return delta * as<float, micro>; }

array<float, 2> MotorStatus::getCurrent() {
  return {lCur * currentFactor, rCur * currentFactor};
}

float currentToTorque(float x) {
  return x * 1.05;
}

array<float, 2> MotorStatus::getTorque() {
  auto current = getCurrent();
  return {currentToTorque(current[0]), currentToTorque(current[1])};
}

array<float, 2> MotorStatus::getVelocity() {
  return {lVel * velocityFactor, rVel * velocityFactor};
}

array<float, 2> MotorStatus::getPosition() {
  return {lPos * positionFactor, rPos * positionFactor};
}
