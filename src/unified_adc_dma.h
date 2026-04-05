#pragma once

#include <cstdint>

namespace firmware {

struct UnifiedAdcStats {
	uint32_t pot_mux_switch_count;
	uint32_t pot_settle_discard_count;
};

class UnifiedAdcDma {
	public:
	UnifiedAdcDma();

	bool init(float audio_sample_rate_hz);
	bool start();
	void stop();

	void poll();
	uint16_t latest_audio_raw_u12() const;
	uint16_t pot_raw_u8(uint8_t pot_index) const;
	UnifiedAdcStats get_stats() const;

	private:
	static const uint8_t kAudioAdcChannel = 1;  // GPIO 27 / ADC1
	static const uint8_t kPotAdcChannel = 0;    // GPIO 26 / ADC0
	static const uint8_t kPotCount = 3;
	static const uint8_t kAudioAverageTaps = 4;
	static const uint16_t kAudioMidRaw = 2048;
	static const uint16_t kAdcMaxRaw = 4095;
	static const uint16_t kPotMaxRaw = 255;
	static const uint16_t kPotSamplesPerHold = 64;
	static const uint8_t kPotDiscardAfterSwitch = 6;
	static const uint32_t kRingSampleCount = 256;
	static const uint32_t kDmaTransferCount = 0xFFFFFFFFu;
	static const uint32_t kRingBufferBytes = kRingSampleCount * sizeof(uint16_t);
	static const uint8_t kRingWrapBits = 9;  // 2^9 = 512 bytes

	void configure_adc_clock(float audio_sample_rate_hz);
	void start_streaming_locked();
	void process_audio_sample_locked(uint16_t raw_u12);
	void process_pot_sample_locked(uint16_t raw_u12);
	void set_active_pot_mux_locked(uint8_t pot_index);

	int dma_channel_;
	bool initialized_;
	bool running_;
	uint32_t dma_read_index_;
	bool expect_pot_sample_;

	volatile uint16_t held_audio_raw_u12_;
	volatile uint16_t pot_raw_u12_[kPotCount];
	volatile uint32_t pot_mux_switch_count_;
	volatile uint32_t pot_settle_discard_count_;

	uint8_t active_pot_index_;
	uint8_t pot_discard_remaining_;
	uint32_t pot_accumulator_;
	uint16_t pot_accumulator_count_;

	uint16_t audio_history_[kAudioAverageTaps];
	uint32_t audio_history_sum_;
	uint8_t audio_history_index_;

	alignas(512) volatile uint16_t ring_buffer_[kRingSampleCount];
};

}  // namespace firmware
