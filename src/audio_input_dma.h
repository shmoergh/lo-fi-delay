#pragma once

#include <cstdint>

namespace firmware {

class AudioInputDma {
	public:
	AudioInputDma();

	bool init(float target_sample_rate_hz);
	bool start();
	void stop();

	void pause_for_control();
	void resume_after_control();

	uint16_t latest_raw_u12();

	private:
	static const uint8_t kAudioAdcChannel = 1;	// GPIO 27 / ADC1
	static const uint32_t kRingSampleCount = 64;
	static const uint32_t kDmaTransferCount = 0xFFFFFFFFu;
	static const uint8_t kPostResumeDiscardSamples = 4;
	static const uint8_t kOutputAverageTaps = 4;

	void configure_adc_clock(float target_sample_rate_hz);
	void start_streaming_locked();

	int dma_channel_;
	bool initialized_;
	bool running_;
	bool paused_;
	volatile uint8_t settle_discard_remaining_;
	volatile uint16_t held_raw_u12_;
	alignas(128) volatile uint16_t ring_buffer_[kRingSampleCount];
};

}  // namespace firmware
