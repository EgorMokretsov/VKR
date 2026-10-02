#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace clover_safety {

// Only the callback that acquired the flag may release it. ROS callbacks can
// re-enter through spinOnce() even with a single-threaded spinner.
class BusyGuard {
	bool& busy_;
public:
	explicit BusyGuard(bool& busy): busy_(busy) {
		if (busy_) throw std::runtime_error("Busy");
		busy_ = true;
	}
	~BusyGuard() { busy_ = false; }
	BusyGuard(const BusyGuard&) = delete;
	BusyGuard& operator=(const BusyGuard&) = delete;
};

inline double navigationProgress(double distance, double speed, double elapsed)
{
	if (!std::isfinite(distance) || !std::isfinite(speed) || !std::isfinite(elapsed) ||
	    distance < 0 || speed <= 0) {
		throw std::runtime_error("Invalid navigation distance, speed or time");
	}
	if (distance <= 1e-6) return 1.0;
	return std::max(0.0, std::min(elapsed * speed / distance, 1.0));
}

struct ControlMessage { int16_t x, y, z, r; };

// The existing Android/iOS wire format is four little-endian signed int16s.
// Decode explicitly: never reinterpret an incomplete or unaligned buffer.
inline bool decodeControl(const unsigned char* data, size_t size, ControlMessage& msg)
{
	if (size != 8) return false;
	int16_t values[4];
	for (size_t i = 0; i < 4; ++i) {
		int value = data[2 * i] | (static_cast<int>(data[2 * i + 1]) << 8);
		if (value >= 32768) value -= 65536;
		if (value < (i == 2 ? 0 : -1000) || value > 1000) return false;
		values[i] = static_cast<int16_t>(value);
	}
	msg = {values[0], values[1], values[2], values[3]};
	return true;
}

} // namespace clover_safety
