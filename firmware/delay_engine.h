#pragma once

#include <cstdint>

#include <pico/time.h>

#include "fast_dac_out.h"

namespace firmware {

struct DelayParams {
	uint32_t delay_samples;
	int16_t feedback_q15;
	int16_t mix_q15;
	int16_t tone_q15;
	bool freeze;
};

class DelayEngine {
	public:
	static const int kAudioPeriodUs = 42;
	static const uint32_t kMaxDelaySamples = 96000;
	static const int16_t kQ15Max = 32767;
	static const int16_t kFeedbackMaxQ15 = 30145;
	static const uint32_t kDelaySlewQ16PerSample = (1u << 16);

	DelayEngine();

	bool init();
	bool start();
	void stop();
	void clear_and_restart();

	void set_params(const DelayParams& params);
	DelayParams get_params() const;
	void set_control_adc_lock(bool locked);

	uint32_t get_overrun_count() const;
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
	volatile uint16_t last_audio_adc_raw_;

	int16_t delay_buffer_[kMaxDelaySamples];
	uint32_t write_index_;
	uint32_t current_delay_q16_;
	int32_t tone_lp_q15_;
};

}  // namespace firmware
