#include "delay_app.h"

#include <pico/stdlib.h>

#include <cmath>
#include <cstdlib>
#include <cstdio>

namespace firmware {

namespace {

template <typename T>
T clamp_value(T v, T lo, T hi) {
	if (v < lo) return lo;
	if (v > hi) return hi;
	return v;
}

float random_unit() {
	return static_cast<float>(rand() & 0x7FFF) / 32767.0f;
}

}  // namespace

DelayApp::DelayApp() :
	brain_(),
	smoothed_delay_ms_(450.0f),
	smoothed_feedback_norm_(0.35f),
	smoothed_mix_norm_(0.45f),
	last_led_update_us_(0),
	next_tempo_pulse_us_(0),
	tempo_pulse_off_us_(0),
	tempo_pulse_on_(false),
	freeze_pressed_(false),
	clear_requested_(false),
	ignore_next_tap_(false),
	tap_time_active_(false),
	tap_pickup_pot_raw_(0),
	last_tap_ms_(0),
	tap_delay_ms_(450.0f) {
	for (uint8_t i = 0; i < kPotCount; i++) {
		pot_values_[i] = 0;
		stable_pot_values_[i] = 0;
	}
	for (uint8_t i = 0; i < kPanelLedCount; i++) {
		led_phase_[i] = 0.0f;
		led_rate_[i] = 0.0f;
		led_target_[i] = 0.0f;
		led_level_[i] = 0.0f;
		led_next_target_us_[i] = 0;
	}
}

bool DelayApp::init() {
	if (kEnableLogging) {
		stdio_init_all();
	}

	const uint32_t now_us = time_us_32();
	last_led_update_us_ = now_us;
	srand(now_us);

	for (uint8_t i = 0; i < kPotCount; i++) {
		pot_values_[i] = kPotMaxRaw / 2;
		stable_pot_values_[i] = pot_values_[i];
	}
	sync_smoothed_controls_from_stable_pots();
	tap_pickup_pot_raw_ = stable_pot_values_[0];

	if (!brain_init_succeeded(brain_.init_buttons(true))) {
		return false;
	}
	if (!brain_init_succeeded(brain_.init_leds(LedMode::kPwm))) {
		return false;
	}
	brain_.leds.off_all();
	brain_.leds.button_init();
	brain_.leds.button_off();
	init_led_animation(now_us);
	wire_button_callbacks();

	if (!engine_.init()) {
		return false;
	}
	if (!engine_.start()) {
		return false;
	}

	sleep_us(30000);
	read_all_pots();
	for (uint8_t i = 0; i < kPotCount; i++) {
		stable_pot_values_[i] = pot_values_[i];
	}
	sync_smoothed_controls_from_stable_pots();
	tap_pickup_pot_raw_ = stable_pot_values_[0];
	next_tempo_pulse_us_ = time_us_32() + tempo_pulse_interval_us(smoothed_delay_ms_);
	tempo_pulse_off_us_ = 0;
	tempo_pulse_on_ = false;

	if (kEnableLogging) {
		printf("lo-fi-delay started (ISR @ ~%.1f Hz, max delay %lu samples)\n",
			static_cast<double>(engine_.sample_rate_hz()),
			static_cast<unsigned long>(DelayEngine::kMaxDelaySamples));
		if (DelayEngine::kTestMode == DelayEngine::AudioTestMode::kDacMidpoint) {
			printf("audio test mode: DAC midpoint\n");
		} else if (DelayEngine::kTestMode == DelayEngine::AudioTestMode::kDryPass) {
			printf("audio test mode: dry pass\n");
		} else {
			printf("audio test mode: normal\n");
		}
	}
	return true;
}

void DelayApp::run() {
	uint32_t last_control_us = time_us_32();
	uint32_t last_debug_us = last_control_us;

	while (true) {
		brain_.update_buttons();
		brain_.update_leds();

		const uint32_t now_us = time_us_32();

		if (clear_requested_) {
			clear_requested_ = false;
			engine_.clear_and_restart();
		}

		if ((now_us - last_control_us) >= kControlIntervalUs) {
			last_control_us = now_us;
			update_control_params();
		}

		update_panel_leds(now_us);

		if (kEnableLogging && ((now_us - last_debug_us) >= kDebugIntervalUs)) {
			last_debug_us = now_us;
			const DelayParams params = engine_.get_params();
			const DelayStats stats = engine_.get_stats();
			const float delay_ms = (static_cast<float>(params.delay_samples) * 1000.0f) /
				engine_.sample_rate_hz();
			const float fb =
				static_cast<float>(params.feedback_q15) / static_cast<float>(DelayEngine::kQ15Max);
			const float mix =
				static_cast<float>(params.mix_q15) / static_cast<float>(DelayEngine::kQ15Max);

			printf(
				"delay=%.1fms fb=%.2f mix=%.2f p0=%u p1=%u p2=%u freeze=%d tap=%d over=%lu"
				" ticks=%lu mux_sw=%lu mux_discard=%lu\n",
				static_cast<double>(delay_ms),
				static_cast<double>(fb),
				static_cast<double>(mix),
				static_cast<unsigned>(stable_pot_values_[0]),
				static_cast<unsigned>(stable_pot_values_[1]),
				static_cast<unsigned>(stable_pot_values_[2]),
				params.freeze ? 1 : 0,
				tap_time_active_ ? 1 : 0,
				static_cast<unsigned long>(stats.overrun_count),
				static_cast<unsigned long>(stats.audio_tick_count),
				static_cast<unsigned long>(stats.pot_mux_switch_count),
				static_cast<unsigned long>(stats.pot_settle_discard_count));
		}

		sleep_us(200);
	}
}

void DelayApp::on_tap_tempo() {
	if (ignore_next_tap_) {
		ignore_next_tap_ = false;
		return;
	}

	const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
	if (last_tap_ms_ > 0) {
		const uint32_t delta_ms = now_ms - last_tap_ms_;
		if (delta_ms >= kTapMinMs) {
			tap_delay_ms_ =
				clamp_value<float>(static_cast<float>(delta_ms), kMinDelayMs, kMaxDelayMs);
			tap_time_active_ = true;
			tap_pickup_pot_raw_ = delay_ms_to_pot_raw(tap_delay_ms_);
		}
	}
	last_tap_ms_ = now_ms;
}

void DelayApp::on_clear_long_press() {
	ignore_next_tap_ = true;
	clear_requested_ = true;
}

void DelayApp::on_freeze_press() {
	freeze_pressed_ = true;
}

void DelayApp::on_freeze_release() {
	freeze_pressed_ = false;
}

void DelayApp::wire_button_callbacks() {
	brain_.buttons.button_a.set_on_single_tap([this]() { this->on_tap_tempo(); });
	brain_.buttons.button_a.set_on_long_press([this]() { this->on_clear_long_press(); });
	brain_.buttons.button_b.set_on_press([this]() { this->on_freeze_press(); });
	brain_.buttons.button_b.set_on_release([this]() { this->on_freeze_release(); });
}

void DelayApp::init_led_animation(uint32_t now_us) {
	for (uint8_t i = 0; i < kPanelLedCount; i++) {
		led_phase_[i] = random_unit() * 6.2831853f;
		led_rate_[i] = 0.35f + random_unit() * 1.2f;
		led_target_[i] = 0.12f + random_unit() * 0.65f;
		led_level_[i] = led_target_[i] * 0.5f;
		led_next_target_us_[i] =
			now_us +
			static_cast<uint32_t>(
				static_cast<float>(kLedTargetMinHoldUs) +
				(static_cast<float>(kLedTargetMaxHoldUs - kLedTargetMinHoldUs) * random_unit()));
	}
}

void DelayApp::update_panel_leds(uint32_t now_us) {
	const uint32_t elapsed_us = now_us - last_led_update_us_;
	if (elapsed_us >= kLedUpdateIntervalUs) {
		const float dt = static_cast<float>(elapsed_us) / 1000000.0f;
		last_led_update_us_ = now_us;

		const float feedback_norm =
			static_cast<float>(stable_pot_values_[1]) / static_cast<float>(kPotMaxRaw);
		const float mix_norm = static_cast<float>(stable_pot_values_[2]) / static_cast<float>(kPotMaxRaw);
		const float activity = 0.35f + 0.65f * ((feedback_norm + mix_norm) * 0.5f);

		for (uint8_t i = 0; i < kPanelLedCount; i++) {
			if (static_cast<int32_t>(now_us - led_next_target_us_[i]) >= 0) {
				led_target_[i] = clamp_value<float>(
					0.10f + (0.80f * random_unit()) * activity, 0.06f, 0.92f);
				led_rate_[i] = 0.35f + random_unit() * (1.25f + activity);
				led_next_target_us_[i] =
					now_us +
					static_cast<uint32_t>(
						static_cast<float>(kLedTargetMinHoldUs) +
						(static_cast<float>(kLedTargetMaxHoldUs - kLedTargetMinHoldUs) * random_unit()));
			}

			led_phase_[i] += dt * led_rate_[i];
			if (led_phase_[i] > 6.2831853f) {
				led_phase_[i] -= 6.2831853f;
			}

			const float wave = 0.5f + 0.5f * sinf(led_phase_[i] + (static_cast<float>(i) * 0.9f));
			const float desired =
				clamp_value<float>(0.04f + (0.54f * led_target_[i]) + (0.42f * wave * activity), 0.0f, 1.0f);
			const float alpha = (desired > led_level_[i]) ? 0.24f : 0.08f;
			led_level_[i] += (desired - led_level_[i]) * alpha;
			led_level_[i] = clamp_value<float>(led_level_[i], 0.0f, 1.0f);

			const float perceptual = led_level_[i] * led_level_[i];
			const uint8_t brightness =
				static_cast<uint8_t>(clamp_value<int32_t>(static_cast<int32_t>(perceptual * 255.0f), 0, 255));
			brain_.leds.set_brightness(i, brightness);
		}
	}

	if (freeze_pressed_) {
		brain_.leds.button_on();
		return;
	}

	if (now_us >= next_tempo_pulse_us_) {
		next_tempo_pulse_us_ = now_us + tempo_pulse_interval_us(smoothed_delay_ms_);
		tempo_pulse_off_us_ = now_us + kTempoPulseOnUs;
		tempo_pulse_on_ = true;
	}
	if (tempo_pulse_on_ && now_us >= tempo_pulse_off_us_) {
		tempo_pulse_on_ = false;
	}
	if (tempo_pulse_on_) {
		brain_.leds.button_on();
	} else {
		brain_.leds.button_off();
	}
}

uint32_t DelayApp::tempo_pulse_interval_us(float delay_ms) const {
	const float clamped_ms = clamp_value<float>(delay_ms, 1.0f, 4000.0f);
	const uint32_t interval_us = static_cast<uint32_t>(clamped_ms * 1000.0f + 0.5f);
	return clamp_value<uint32_t>(
		interval_us, kTempoPulseMinIntervalUs, kTempoPulseMaxIntervalUs);
}

void DelayApp::update_control_params() {
	// UnifiedAdcDma already handles mux settle/averaging; here we only consume snapshots.
	read_all_pots();

	const auto apply_deadband = [](uint16_t incoming, uint16_t current, uint8_t deadband) -> uint16_t {
		const int32_t delta = static_cast<int32_t>(incoming) - static_cast<int32_t>(current);
		const int32_t abs_delta = (delta < 0) ? -delta : delta;
		if (abs_delta <= static_cast<int32_t>(deadband)) {
			return current;
		}
		return incoming;
	};
	stable_pot_values_[0] =
		apply_deadband(pot_values_[0], stable_pot_values_[0], kPotDeadbandTime);
	stable_pot_values_[1] =
		apply_deadband(pot_values_[1], stable_pot_values_[1], kPotDeadbandFeedback);
	stable_pot_values_[2] =
		apply_deadband(pot_values_[2], stable_pot_values_[2], kPotDeadbandMix);

	if (tap_time_active_) {
		const int32_t delta =
			static_cast<int32_t>(stable_pot_values_[0]) - static_cast<int32_t>(tap_pickup_pot_raw_);
		const int32_t abs_delta = (delta < 0) ? -delta : delta;
		if (abs_delta <= static_cast<int32_t>(kTapPickupThreshold)) {
			tap_time_active_ = false;
		}
	}

	const float pot_delay_ms = map_time_pot_to_ms(stable_pot_values_[0]);
	const float target_delay_ms = tap_time_active_ ? tap_delay_ms_ : pot_delay_ms;
	smoothed_delay_ms_ += (target_delay_ms - smoothed_delay_ms_) * kDelaySmoothingAlpha;
	if (fabsf(target_delay_ms - smoothed_delay_ms_) < 0.5f) {
		smoothed_delay_ms_ = target_delay_ms;
	}

	const float feedback_norm =
		static_cast<float>(stable_pot_values_[1]) / static_cast<float>(kPotMaxRaw);
	const float mix_norm = static_cast<float>(stable_pot_values_[2]) / static_cast<float>(kPotMaxRaw);
	smoothed_feedback_norm_ += (feedback_norm - smoothed_feedback_norm_) * kFeedbackSmoothingAlpha;
	smoothed_mix_norm_ += (mix_norm - smoothed_mix_norm_) * kMixSmoothingAlpha;
	if (fabsf(feedback_norm - smoothed_feedback_norm_) < 0.004f) {
		smoothed_feedback_norm_ = feedback_norm;
	}
	if (fabsf(mix_norm - smoothed_mix_norm_) < 0.004f) {
		smoothed_mix_norm_ = mix_norm;
	}

	DelayParams params = engine_.get_params();
	params.delay_samples = delay_ms_to_samples(smoothed_delay_ms_);
	params.feedback_q15 = to_q15(smoothed_feedback_norm_);
	if (params.feedback_q15 > DelayEngine::kFeedbackMaxQ15) {
		params.feedback_q15 = DelayEngine::kFeedbackMaxQ15;
	}
	params.mix_q15 = to_q15(smoothed_mix_norm_);
	params.tone_q15 = static_cast<int16_t>(0.22f * static_cast<float>(DelayEngine::kQ15Max));
	params.freeze = freeze_pressed_;
	engine_.set_params(params);
}

void DelayApp::read_all_pots() {
	for (uint8_t i = 0; i < kPotCount; i++) {
		pot_values_[i] = engine_.read_pot_raw_u8(i);
	}
}

void DelayApp::sync_smoothed_controls_from_stable_pots() {
	smoothed_delay_ms_ = map_time_pot_to_ms(stable_pot_values_[0]);
	smoothed_feedback_norm_ =
		static_cast<float>(stable_pot_values_[1]) / static_cast<float>(kPotMaxRaw);
	smoothed_mix_norm_ =
		static_cast<float>(stable_pot_values_[2]) / static_cast<float>(kPotMaxRaw);
}

float DelayApp::map_time_pot_to_ms(uint16_t raw) const {
	const float norm = static_cast<float>(raw) / static_cast<float>(kPotMaxRaw);
	const float ratio = kMaxDelayMs / kMinDelayMs;
	return kMinDelayMs * powf(ratio, norm);
}

uint16_t DelayApp::delay_ms_to_pot_raw(float delay_ms) const {
	const float clamped_ms = clamp_value<float>(delay_ms, kMinDelayMs, kMaxDelayMs);
	const float ratio = kMaxDelayMs / kMinDelayMs;
	const float norm = logf(clamped_ms / kMinDelayMs) / logf(ratio);
	const float clamped_norm = clamp_value<float>(norm, 0.0f, 1.0f);
	const float raw = clamped_norm * static_cast<float>(kPotMaxRaw);
	return static_cast<uint16_t>(raw + 0.5f);
}

int16_t DelayApp::to_q15(float norm01) const {
	const float clamped = clamp_value<float>(norm01, 0.0f, 1.0f);
	return static_cast<int16_t>(clamped * static_cast<float>(DelayEngine::kQ15Max));
}

uint32_t DelayApp::delay_ms_to_samples(float delay_ms) const {
	const float clamped_ms = clamp_value<float>(delay_ms, kMinDelayMs, kMaxDelayMs);
	const uint32_t samples = static_cast<uint32_t>(
		(clamped_ms * engine_.sample_rate_hz()) / 1000.0f + 0.5f);
	return clamp_value<uint32_t>(samples, 1, DelayEngine::kMaxDelaySamples - 1);
}

}  // namespace firmware
