#pragma once

#include <cstdint>

namespace firmware {

class FastDacOut {
	public:
	static const uint32_t kSpiFrequencyHz = 1000000;

	bool init();
	void write_channel_a_raw(uint16_t raw12);
};

}  // namespace firmware
