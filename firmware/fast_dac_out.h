#pragma once

#include <cstdint>

namespace firmware {

class FastDacOut {
	public:
	bool init();
	void write_channel_a_raw(uint16_t raw12);
};

}  // namespace firmware
