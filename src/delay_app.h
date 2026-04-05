#pragma once

#include <cstdint>

#include "brain/brain.h"
#include "delay_engine.h"

namespace firmware {

class DelayApp {
	public:
	DelayApp();

	bool init();
	void run();

	private:
	static const uint8_t kPotCount = 3;
	static const uint32_t kControlIntervalUs = 20000;
	static const uint32_t kPotReadIntervalActiveUs = 90000;
	static const uint32_t kPotReadIntervalIdleUs = 300000;
	static const uint32_t kPotActivityHoldUs = 1400000;
	static const uint8_t kPotActivityDetectThreshold = 2;
	static const uint32_t kDebugIntervalUs = 300000;
	static const bool kEnableLogging = false;
	static const uint32_t kLedUpdateIntervalUs = 20000;
	static const uint16_t kTapPickupThreshold = 64;
	static const uint32_t kTapMinMs = 80;
	static const bool kEnableTapTempo = true;
	static const bool kEnableTempoPulseLed = true;
	static const bool kEnablePotPolling = true;
	static const uint16_t kPotMaxRaw = 255;
	static constexpr float kMinDelayMs = 30.0f;
	static constexpr float kMaxDelayMs = 1000.0f;

	static const uint8_t kPotDeadbandTime = 2;
	static const uint8_t kPotDeadbandFeedback = 1;
	static const uint8_t kPotDeadbandMix = 1;
	static constexpr float kDelaySmoothingAlpha = 0.08f;
	static constexpr float kFeedbackSmoothingAlpha = 0.18f;
	static constexpr float kMixSmoothingAlpha = 0.18f;
	static const uint8_t kPanelLedCount = NO_OF_LEDS;
	static const uint32_t kTempoPulseOnUs = 42000;
	static const uint32_t kTempoPulseMinIntervalUs = 120000;
	static const uint32_t kTempoPulseMaxIntervalUs = 1200000;
	static const uint32_t kLedTargetMinHoldUs = 260000;
	static const uint32_t kLedTargetMaxHoldUs = 1300000;

	void on_tap_tempo();
	void on_clear_long_press();
	void on_freeze_press();
	void on_freeze_release();
	void update_panel_leds(uint32_t now_us);
	uint32_t tempo_pulse_interval_us(float delay_ms) const;

	void update_control_params();
	float map_time_pot_to_ms(uint16_t raw) const;
	uint16_t delay_ms_to_pot_raw(float delay_ms) const;
	int16_t to_q15(float norm01) const;
	uint32_t delay_ms_to_samples(float delay_ms) const;

	Brain brain_;
	DelayEngine engine_;

	uint16_t pot_values_[kPotCount];
	uint16_t stable_pot_values_[kPotCount];
	uint8_t next_pot_index_;
	float smoothed_delay_ms_;
	float smoothed_feedback_norm_;
	float smoothed_mix_norm_;
	uint32_t last_pot_read_us_;
	uint32_t pot_active_until_us_;
	uint32_t last_led_update_us_;
	uint32_t next_tempo_pulse_us_;
	uint32_t tempo_pulse_off_us_;
	bool tempo_pulse_on_;
	float led_phase_[kPanelLedCount];
	float led_rate_[kPanelLedCount];
	float led_target_[kPanelLedCount];
	float led_level_[kPanelLedCount];
	uint32_t led_next_target_us_[kPanelLedCount];

	bool freeze_pressed_;
	bool clear_requested_;
	bool ignore_next_tap_;
	bool tap_time_active_;
	uint16_t tap_pickup_pot_raw_;
	uint32_t last_tap_ms_;
	float tap_delay_ms_;
};

}  // namespace firmware
