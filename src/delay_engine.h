#pragma once

#include <cstdint>

#include "brain/brain.h"

namespace firmware {

struct DelayParams {
	uint32_t delay_samples;
	int16_t feedback_q15;
	int16_t mix_q15;
	int16_t tone_q15;
	bool freeze;
};

struct DelayStats {
	uint32_t audio_tick_count;
	uint32_t pot_mux_switch_count;
	uint32_t pot_settle_discard_count;
	uint32_t overrun_count;
};

class DelayEngine {
public:
	enum class AudioTestMode : uint8_t {
		kNormal = 0,
		kDacMidpoint = 1,
		kDryPass = 2
	};

	static const int kAudioPeriodUs = 23;
	// Power of two so the ring buffer index uses a bitmask instead of a divide.
	// 65536 samples * 23 us = ~1.5 s of headroom for the 1000 ms max delay.
	static const uint32_t kMaxDelaySamples = 65536;
	static const uint32_t kDelayIndexMask = kMaxDelaySamples - 1;
	static const int16_t kQ15Max = 32767;
	static const int16_t kFeedbackMaxQ15 = 30145;
	static const uint32_t kDelaySlewQ16PerSample = (1u << 12);
	static constexpr AudioTestMode kTestMode = AudioTestMode::kNormal;
	static const int16_t kDcBlockCoeffQ15 = 32604;  // ~0.995

	DelayEngine();

	bool init(Brain& brain);
	bool start();
	void stop();
	void clear_and_restart();

	void set_params(const DelayParams& params);
	DelayParams get_params() const;
	uint16_t read_pot_raw_u8(uint8_t pot_index) const;

	DelayStats get_stats() const;
	float sample_rate_hz() const;

private:
	bool try_start_audio_processor(const AudioProcessorConfig& config);
	static int16_t audio_callback(
		int16_t input_sample,
		const AudioProcessorFrame* frame,
		void* user_ctx);
	int16_t process_audio_sample(int16_t input_sample, const AudioProcessorFrame* frame);
	int16_t process_delay_sample(const DelayParams& params, int16_t input_sample);

	Brain* brain_;
	bool initialized_;
	bool running_;

	DelayParams params_;
	int16_t dc_block_x_prev_;
	int32_t dc_block_y_prev_;
	int16_t prev_input_raw_sample_;

	int16_t delay_buffer_[kMaxDelaySamples];
	uint32_t write_index_;
	uint32_t current_delay_q16_;
	int32_t tone_lp_q15_;

	// AudioProcessor pot scanning tuning (consumed by the AudioProcessorConfig in start()).
	static const uint8_t kAdcPotCount = 3;
	static const uint16_t kAdcPotMaxRaw = 255;
	static const uint8_t kAdcPotSamplesPerHold = 4;
	static const uint8_t kAdcPotDiscardAfterSwitch = 2;
};

}  // namespace firmware
