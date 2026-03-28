#include "delay_app.h"

#include <hardware/sync.h>
#include <pico/stdlib.h>

#include <cmath>
#include <cstdio>

#include "brain-common/brain-common.h"

namespace firmware {

namespace {

template <typename T>
T clamp_value(T v, T lo, T hi) {
	if (v < lo) return lo;
	if (v > hi) return hi;
	return v;
}

}  // namespace

DelayApp::DelayApp() :
	button_tap_clear_(BRAIN_BUTTON_1, 30, kButtonLongPressMs),
	button_freeze_(BRAIN_BUTTON_2, 30, 500),
	overrun_led_(BRAIN_LED_1, true),
	next_pot_index_(0),
	smoothed_delay_ms_(450.0f),
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
}

bool DelayApp::init() {
	stdio_init_all();

	brain::ui::PotsConfig pot_cfg = brain::ui::create_default_config(3, 8);
	pot_cfg.simple = true;
	pot_cfg.samples_per_read = 1;
	pot_cfg.settling_delay_us = 50;
	pot_cfg.change_threshold = 0;
	pots_.init(pot_cfg);
	for (uint8_t i = 0; i < kPotCount; i++) {
		pot_values_[i] = read_pot_atomic(i);
	}

	for (uint8_t i = 0; i < kPotCount; i++) {
		stable_pot_values_[i] = pot_values_[i];
	}
	smoothed_delay_ms_ = map_time_pot_to_ms(stable_pot_values_[0]);
	tap_pickup_pot_raw_ = pot_values_[0];

	button_tap_clear_.init(true);
	button_freeze_.init(true);
	button_led_.init();
	overrun_led_.init(brain::ui::LedMode::kSimple);

	if (kEnableTapTempo) {
		button_tap_clear_.set_on_single_tap([this]() { this->on_tap_tempo(); });
	}
	button_tap_clear_.set_on_long_press([this]() { this->on_clear_long_press(); });

	button_freeze_.set_on_press([this]() { this->on_freeze_press(); });
	button_freeze_.set_on_release([this]() { this->on_freeze_release(); });

	if (!engine_.init()) {
		return false;
	}
	if (!engine_.start()) {
		return false;
	}

	printf("lo-fi-delay started (ISR @ ~%.1f Hz, max delay %lu samples)\n",
		static_cast<double>(engine_.sample_rate_hz()),
		static_cast<unsigned long>(DelayEngine::kMaxDelaySamples));
	return true;
}

void DelayApp::run() {
	uint32_t last_control_us = time_us_32();
	uint32_t last_debug_us = last_control_us;

	while (true) {
		button_tap_clear_.update();
		button_freeze_.update();
		button_led_.update();

		const uint32_t now_us = time_us_32();

		if (clear_requested_) {
			clear_requested_ = false;
			engine_.clear_and_restart();
		}

		if ((now_us - last_control_us) >= kControlIntervalUs) {
			last_control_us = now_us;
			update_control_params();
		}

		if (freeze_pressed_) {
			button_led_.on();
		} else {
			button_led_.off();
		}

		if (engine_.get_overrun_count() > 0) {
			overrun_led_.on();
		} else {
			overrun_led_.off();
		}

		if ((now_us - last_debug_us) >= kDebugIntervalUs) {
			last_debug_us = now_us;
			const DelayParams params = engine_.get_params();
			const float delay_ms = (static_cast<float>(params.delay_samples) * 1000.0f) /
				engine_.sample_rate_hz();
			const float fb =
				static_cast<float>(params.feedback_q15) / static_cast<float>(DelayEngine::kQ15Max);
			const float mix =
				static_cast<float>(params.mix_q15) / static_cast<float>(DelayEngine::kQ15Max);

			printf("delay=%.1fms fb=%.2f mix=%.2f p0=%u p1=%u p2=%u freeze=%d tap=%d overruns=%lu\n",
				static_cast<double>(delay_ms),
				static_cast<double>(fb),
				static_cast<double>(mix),
				static_cast<unsigned>(stable_pot_values_[0]),
				static_cast<unsigned>(stable_pot_values_[1]),
				static_cast<unsigned>(stable_pot_values_[2]),
				params.freeze ? 1 : 0,
				tap_time_active_ ? 1 : 0,
				static_cast<unsigned long>(engine_.get_overrun_count()));
		}

		sleep_us(200);
	}
}

void DelayApp::on_tap_tempo() {
	if (!kEnableTapTempo) {
		return;
	}

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

void DelayApp::update_control_params() {
	pot_values_[next_pot_index_] = read_pot_atomic(next_pot_index_);
	next_pot_index_ = static_cast<uint8_t>((next_pot_index_ + 1) % kPotCount);

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

	if (kEnableTapTempo && tap_time_active_) {
		const int32_t delta =
			static_cast<int32_t>(stable_pot_values_[0]) - static_cast<int32_t>(tap_pickup_pot_raw_);
		const int32_t abs_delta = (delta < 0) ? -delta : delta;
		if (abs_delta <= static_cast<int32_t>(kTapPickupThreshold)) {
			tap_time_active_ = false;
		}
	}

	const float pot_delay_ms = map_time_pot_to_ms(stable_pot_values_[0]);
	const float target_delay_ms = (kEnableTapTempo && tap_time_active_) ? tap_delay_ms_ : pot_delay_ms;
	smoothed_delay_ms_ += (target_delay_ms - smoothed_delay_ms_) * kDelaySmoothingAlpha;
	if (fabsf(target_delay_ms - smoothed_delay_ms_) < 0.5f) {
		smoothed_delay_ms_ = target_delay_ms;
	}

	const float feedback_norm =
		static_cast<float>(stable_pot_values_[1]) / static_cast<float>(kPotMaxRaw);
	const float mix_norm = static_cast<float>(stable_pot_values_[2]) / static_cast<float>(kPotMaxRaw);

	DelayParams params = engine_.get_params();
	params.delay_samples = delay_ms_to_samples(smoothed_delay_ms_);
	params.feedback_q15 = to_q15(feedback_norm);
	if (params.feedback_q15 > DelayEngine::kFeedbackMaxQ15) {
		params.feedback_q15 = DelayEngine::kFeedbackMaxQ15;
	}
	params.mix_q15 = to_q15(mix_norm);
	params.tone_q15 = static_cast<int16_t>(0.22f * static_cast<float>(DelayEngine::kQ15Max));
	params.freeze = freeze_pressed_;
	engine_.set_params(params);
}

uint16_t DelayApp::read_pot_atomic(uint8_t index) {
	const uint32_t irq_state = save_and_disable_interrupts();
	const uint16_t value = pots_.get(index);
	restore_interrupts(irq_state);
	return value;
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
