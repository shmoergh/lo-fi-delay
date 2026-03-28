#pragma once

#include <cstdint>

#include <pico/time.h>

#include "audio_input_dma.h"
#include "fast_dac_out.h"

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
	uint32_t adc_stale_sample_count;
	uint32_t control_lock_events;
	uint64_t control_lock_total_us;
	uint32_t control_lock_max_us;
	uint32_t overrun_count;
};

class DelayEngine {
	public:
	enum class AudioTestMode : uint8_t {
		kNormal = 0,
		kDacMidpoint = 1,
		kDryPass = 2
	};

	static const int kAudioPeriodUs = 42;
	static const uint32_t kMaxDelaySamples = 24000;
	static const int16_t kQ15Max = 32767;
	static const int16_t kFeedbackMaxQ15 = 30145;
	static const uint32_t kDelaySlewQ16PerSample = (1u << 14);
	static constexpr AudioTestMode kTestMode = AudioTestMode::kNormal;
	static const bool kEnableInputAveraging = true;
	static const bool kEnableInputNoiseGate = false;
	static const int16_t kInputGateOpenThreshold = 360;
	static const int16_t kInputGateCloseThreshold = 220;

	DelayEngine();

	bool init();
	bool start();
	void stop();
	void clear_and_restart();

	void set_params(const DelayParams& params);
	DelayParams get_params() const;
	void set_control_adc_lock(bool locked);

	uint32_t get_overrun_count() const;
	DelayStats get_stats() const;
	float sample_rate_hz() const;

	private:
	static bool timer_callback(repeating_timer* timer);
	bool process_audio_tick();

	static DelayEngine* instance_;

	FastDacOut dac_;
	repeating_timer timer_;
	bool running_;

	DelayParams params_;
	volatile uint32_t isr_overrun_count_;
	volatile bool control_adc_locked_;
	volatile uint32_t audio_tick_count_;
	volatile uint32_t adc_stale_sample_count_;
	volatile uint32_t control_lock_events_;
	volatile uint64_t control_lock_total_us_;
	volatile uint32_t control_lock_max_us_;
	volatile uint32_t control_lock_started_us_;
	volatile uint16_t last_audio_adc_raw_;
	int16_t prev_input_raw_sample_;
	bool input_gate_open_;

	AudioInputDma audio_input_dma_;
	int16_t delay_buffer_[kMaxDelaySamples];
	uint32_t write_index_;
	uint32_t current_delay_q16_;
	int32_t tone_lp_q15_;
};

}  // namespace firmware
