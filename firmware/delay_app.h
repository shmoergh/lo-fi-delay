#pragma once

#include <cstdint>

#include "brain-ui/button-led.h"
#include "brain-ui/button.h"
#include "brain-ui/led.h"
#include "brain-ui/pots.h"
#include "delay_engine.h"

namespace firmware {

class DelayApp {
	public:
	DelayApp();

	bool init();
	void run();

	private:
	static const uint8_t kPotCount = 3;
	static const uint32_t kControlIntervalUs = 10000;
	static const uint32_t kDebugIntervalUs = 300000;
	static const uint32_t kButtonLongPressMs = 700;
	static const uint16_t kTapPickupThreshold = 64;
	static const uint32_t kTapMinMs = 80;
	static const bool kEnableTapTempo = false;
	static const uint16_t kPotMaxRaw = 255;
	static constexpr float kMinDelayMs = 30.0f;
	static constexpr float kMaxDelayMs = 4000.0f;

	static const uint8_t kPotDeadbandTime = 2;
	static const uint8_t kPotDeadbandFeedback = 1;
	static const uint8_t kPotDeadbandMix = 1;
	static constexpr float kDelaySmoothingAlpha = 0.18f;

	void on_tap_tempo();
	void on_clear_long_press();
	void on_freeze_press();
	void on_freeze_release();

	void update_control_params();
	uint16_t read_pot_atomic(uint8_t index);
	float map_time_pot_to_ms(uint16_t raw) const;
	uint16_t delay_ms_to_pot_raw(float delay_ms) const;
	int16_t to_q15(float norm01) const;
	uint32_t delay_ms_to_samples(float delay_ms) const;

	brain::ui::Button button_tap_clear_;
	brain::ui::Button button_freeze_;
	brain::ui::ButtonLed button_led_;
	brain::ui::Led overrun_led_;

	brain::ui::Pots pots_;
	DelayEngine engine_;

	uint16_t pot_values_[kPotCount];
	uint16_t stable_pot_values_[kPotCount];
	uint8_t next_pot_index_;
	float smoothed_delay_ms_;

	bool freeze_pressed_;
	bool clear_requested_;
	bool ignore_next_tap_;
	bool tap_time_active_;
	uint16_t tap_pickup_pot_raw_;
	uint32_t last_tap_ms_;
	float tap_delay_ms_;
};

}  // namespace firmware
