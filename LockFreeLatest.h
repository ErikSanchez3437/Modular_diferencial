#include <array>
#include <cstdint>
#include <atomic>

using std::array;
using std::atomic;

template <typename T> struct LockFreeLatest {
	static constexpr uint8_t isNewBit = uint8_t(1) << 7;
	array<T, 3> packets;
	atomic<uint8_t> sharedPacket;
	uint8_t freePacket;
	uint8_t latestPacket;

	T& getLatest() {
		bool needUpdate = sharedPacket.load(std::memory_order_acquire) & isNewBit;
		if(needUpdate) {
			latestPacket = sharedPacket.exchange(latestPacket, std::memory_order_acq_rel);
		}
		latestPacket &= ~isNewBit;
		return packets[latestPacket];
	}

	T& getFree() {
		return packets[freePacket];
	}

	void submit() {
		freePacket |= isNewBit;
		freePacket = sharedPacket.exchange(freePacket, std::memory_order_acq_rel);
		freePacket &= ~isNewBit;
	}

	LockFreeLatest(T initialState) : packets{initialState, initialState, initialState}, sharedPacket(0), freePacket(1), latestPacket(2) {}
};
