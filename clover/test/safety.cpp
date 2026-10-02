#include "../src/safety.h"
#include <iostream>
#include <limits>

void require(bool condition, const char* message) {
	if (!condition) throw std::runtime_error(message);
}

int main()
{
	using namespace clover_safety;
	ControlMessage msg{12, 34, 56, 78};
	const unsigned char packet[9] = {0x18, 0xfc, 0xe8, 0x03, 0, 0, 0xff, 0xff, 0};
	for (size_t n = 0; n < 8; ++n) {
		require(!decodeControl(packet, n, msg), "short packet accepted");
		require(msg.x == 12, "rejected packet modified command");
	}
	require(!decodeControl(packet, 9, msg), "oversized packet accepted");
	require(decodeControl(packet, 8, msg), "valid packet rejected");
	require(msg.x == -1000 && msg.y == 1000 && msg.z == 0 && msg.r == -1, "wrong endian/sign decoding");
	const unsigned char invalid[8] = {0xe9, 0x03};
	require(!decodeControl(invalid, 8, msg), "out-of-range control accepted");
	const unsigned char invalid_throttle[8] = {0, 0, 0, 0, 0xff, 0xff, 0, 0};
	require(!decodeControl(invalid_throttle, 8, msg), "negative throttle accepted");
	require(navigationProgress(0, 0.5, 0) == 1, "current target produced NaN");
	require(navigationProgress(1e-9, 0.5, 0) == 1, "near-zero target produced NaN");
	require(navigationProgress(10, 2, 1) == 0.2, "wrong navigation interpolation");
	require(navigationProgress(10, 2, -1) == 0, "backwards time produced negative progress");
	require(navigationProgress(10, 2, 99) == 1, "navigation overshot");
	for (double speed : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN()}) {
		bool rejected = false;
		try { navigationProgress(1, speed, 0); } catch (std::runtime_error&) { rejected = true; }
		require(rejected, "invalid speed accepted");
	}
	bool busy = false;
	{
		BusyGuard first(busy);
		for (int i = 0; i < 2; ++i) {
			bool rejected = false;
			try { BusyGuard nested(busy); } catch (std::runtime_error&) { rejected = true; }
			require(rejected && busy, "nested request released first request's guard");
		}
	}
	require(!busy, "guard was not released");
	try { BusyGuard failure(busy); throw std::runtime_error("failure"); } catch (std::runtime_error&) {}
	require(!busy, "exception leaked guard");
	std::cout << "RC decoder, navigation and reentrant guard checks passed\n";
}
