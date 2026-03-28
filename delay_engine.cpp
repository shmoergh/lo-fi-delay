#include "delay_engine.h"

#include <hardware/sync.h>
#include <pico/time.h>

#include <cstring>

#include "brain-common/brain-common.h"

namespace firmware {

namespace {

template <typename T>
T clamp_value(T v, T lo, T hi) {
	if (v < lo) return lo;
	if (v > hi) return hi;
	return v;
}

int16_t clamp_i16(int32_t v) {
	if (v > 32767) return 32767;
	if (v < -32768) return -32768;
	return static_cast<int16_t>(v);
}

uint16_t sample_to_dac_u12(int16_t sample) {
	int32_t dac = 2048 + ((static_cast<int32_t>(sample) * 2047) / 32768);
	dac = clamp_value<int32_t>(dac, 0, 4095);
	return static_cast<uint16_t>(dac);
}

int16_t soft_clip_i16(int32_t v) {
	const int32_t x = clamp_value<int32_t>(v, -32768, 32767);
	const int32_t abs_x = (x < 0) ? -x : x;
	if (abs_x <= 24576) {
		return static_cast<int16_t>(x);
	}
	int32_t compressed = 24576 + ((abs_x - 24576) >> 2);
	compressed = (compressed > 32767) ? 32767 : compressed;
	return static_cast<int16_t>((x < 0) ? -compressed : compressed);
}

const int32_t kInputLowRaw = static_cast<int32_t>(
	(brain::constants::kAudioCvInVoltageAtMinus5V / brain::constants::kAdcVoltageRef) *
	brain::constants::kAdcMaxValue);
const int32_t kInputHighRaw = static_cast<int32_t>(
	(brain::constants::kAudioCvInVoltageAtPlus5V / brain::constants::kAdcVoltageRef) *
	brain::constants::kAdcMaxValue);
const int32_t kInputMidRaw = (kInputLowRaw + kInputHighRaw) / 2;
const int32_t kInputHalfSpanRaw = ((kInputHighRaw - kInputLowRaw) > 0)
	? ((kInputHighRaw - kInputLowRaw) / 2)
	: 1;

int16_t adc_to_audio_sample(uint16_t adc_raw) {
	const int32_t centered = static_cast<int32_t>(adc_raw) - kInputMidRaw;
	const int32_t scaled = (centered * 32767) / kInputHalfSpanRaw;
	return clamp_i16(scaled);
}

void mark_overrun_if_needed(volatile uint32_t& counter, uint32_t tick_start_us, uint32_t budget_us) {
	const uint32_t elapsed_us = time_us_32() - tick_start_us;
	if (elapsed_us > budget_us) {
		counter++;
	}
}

}  // namespace

DelayEngine* DelayEngine::instance_ = nullptr;

DelayEngine::DelayEngine() :
	running_(false),
	isr_overrun_count_(0),
	control_adc_locked_(false),
	audio_tick_count_(0),
	adc_stale_sample_count_(0),
	control_lock_events_(0),
	control_lock_total_us_(0),
	control_lock_max_us_(0),
	control_lock_started_us_(0),
	last_audio_adc_raw_(2048),
	prev_input_raw_sample_(0),
	input_gate_open_(false),
	write_index_(0),
	current_delay_q16_(1u << 16),
	tone_lp_q15_(0) {
	params_.delay_samples = static_cast<uint32_t>(0.35f * sample_rate_hz());
	params_.feedback_q15 = static_cast<int16_t>(0.35f * static_cast<float>(kQ15Max));
	params_.mix_q15 = static_cast<int16_t>(0.45f * static_cast<float>(kQ15Max));
	params_.tone_q15 = static_cast<int16_t>(0.22f * static_cast<float>(kQ15Max));
	params_.freeze = false;
	current_delay_q16_ = params_.delay_samples << 16;
}

bool DelayEngine::init() {
	if (!dac_.init()) {
		return false;
	}

	memset(delay_buffer_, 0, sizeof(delay_buffer_));
	write_index_ = 0;
	current_delay_q16_ = params_.delay_samples << 16;
	tone_lp_q15_ = 0;
	isr_overrun_count_ = 0;
	audio_tick_count_ = 0;
	adc_stale_sample_count_ = 0;
	control_lock_events_ = 0;
	control_lock_total_us_ = 0;
	control_lock_max_us_ = 0;
	control_lock_started_us_ = 0;
	prev_input_raw_sample_ = 0;
	input_gate_open_ = false;
	last_audio_adc_raw_ = 2048;

	if (!audio_input_dma_.init(sample_rate_hz() * 6.0f)) {
		return false;
	}

	instance_ = this;
	return true;
}

bool DelayEngine::start() {
	if (running_) return true;
	if (!audio_input_dma_.start()) {
		return false;
	}
	running_ = add_repeating_timer_us(-kAudioPeriodUs, DelayEngine::timer_callback, nullptr, &timer_);
	if (!running_) {
		audio_input_dma_.stop();
	}
	return running_;
}

void DelayEngine::stop() {
	if (running_) {
		cancel_repeating_timer(&timer_);
		running_ = false;
	}
	audio_input_dma_.stop();
}

void DelayEngine::clear_and_restart() {
	stop();
	memset(delay_buffer_, 0, sizeof(delay_buffer_));
	write_index_ = 0;
	current_delay_q16_ = params_.delay_samples << 16;
	tone_lp_q15_ = 0;
	prev_input_raw_sample_ = 0;
	input_gate_open_ = false;
	last_audio_adc_raw_ = 2048;
	start();
}

void DelayEngine::set_params(const DelayParams& params) {
	const uint32_t irq_state = save_and_disable_interrupts();
	params_ = params;
	restore_interrupts(irq_state);
}

void DelayEngine::set_control_adc_lock(bool locked) {
	const uint32_t irq_state = save_and_disable_interrupts();

	const uint32_t now_us = time_us_32();
	if (locked) {
		if (!control_adc_locked_) {
			control_lock_started_us_ = now_us;
			audio_input_dma_.pause_for_control();
		}
	} else if (control_adc_locked_) {
		const uint32_t held_us = now_us - control_lock_started_us_;
		control_lock_events_++;
		control_lock_total_us_ += held_us;
		if (held_us > control_lock_max_us_) {
			control_lock_max_us_ = held_us;
		}
		audio_input_dma_.resume_after_control();
	}

	control_adc_locked_ = locked;
	restore_interrupts(irq_state);
}

DelayParams DelayEngine::get_params() const {
	const uint32_t irq_state = save_and_disable_interrupts();
	const DelayParams copy = params_;
	restore_interrupts(irq_state);
	return copy;
}

uint32_t DelayEngine::get_overrun_count() const {
	return isr_overrun_count_;
}

DelayStats DelayEngine::get_stats() const {
	const uint32_t irq_state = save_and_disable_interrupts();
	DelayStats stats{};
	stats.audio_tick_count = audio_tick_count_;
	stats.adc_stale_sample_count = adc_stale_sample_count_;
	stats.control_lock_events = control_lock_events_;
	stats.control_lock_total_us = control_lock_total_us_;
	stats.control_lock_max_us = control_lock_max_us_;
	stats.overrun_count = isr_overrun_count_;
	restore_interrupts(irq_state);
	return stats;
}

float DelayEngine::sample_rate_hz() const {
	return 1000000.0f / static_cast<float>(kAudioPeriodUs);
}

bool DelayEngine::timer_callback(repeating_timer* timer) {
	(void) timer;
	if (instance_ == nullptr) return true;
	return instance_->process_audio_tick();
}

bool DelayEngine::process_audio_tick() {
	const uint32_t tick_start_us = time_us_32();
	audio_tick_count_++;

	if (kTestMode == AudioTestMode::kDacMidpoint) {
		dac_.write_channel_a_raw(2048);
		mark_overrun_if_needed(isr_overrun_count_, tick_start_us, kAudioPeriodUs);
		return true;
	}

	const DelayParams params = params_;

	const uint32_t target_delay_samples =
		clamp_value<uint32_t>(params.delay_samples, 1, kMaxDelaySamples - 2);
	const uint32_t target_delay_q16 = target_delay_samples << 16;
	if (current_delay_q16_ < target_delay_q16) {
		const uint32_t delta = target_delay_q16 - current_delay_q16_;
		const uint32_t step = (delta > kDelaySlewQ16PerSample) ? kDelaySlewQ16PerSample : delta;
		current_delay_q16_ += step;
	} else if (current_delay_q16_ > target_delay_q16) {
		const uint32_t delta = current_delay_q16_ - target_delay_q16;
		const uint32_t step = (delta > kDelaySlewQ16PerSample) ? kDelaySlewQ16PerSample : delta;
		current_delay_q16_ -= step;
	}
	uint32_t delay_int = current_delay_q16_ >> 16;
	delay_int = clamp_value<uint32_t>(delay_int, 1, kMaxDelaySamples - 2);
	const uint32_t frac_q16 = current_delay_q16_ & 0xFFFFu;

	uint16_t adc_raw = last_audio_adc_raw_;
	if (!control_adc_locked_) {
		adc_raw = audio_input_dma_.latest_raw_u12();
		last_audio_adc_raw_ = adc_raw;
	} else {
		adc_stale_sample_count_++;
	}
	int16_t input_sample = adc_to_audio_sample(adc_raw);
	if (kEnableInputAveraging) {
		const int32_t averaged = (static_cast<int32_t>(input_sample) +
								 static_cast<int32_t>(prev_input_raw_sample_)) /
			2;
		prev_input_raw_sample_ = input_sample;
		input_sample = clamp_i16(averaged);
	}
	if (kEnableInputNoiseGate) {
		const int32_t abs_input = (input_sample < 0) ? -static_cast<int32_t>(input_sample)
													 : static_cast<int32_t>(input_sample);
		if (input_gate_open_) {
			if (abs_input <= static_cast<int32_t>(kInputGateCloseThreshold)) {
				input_gate_open_ = false;
			}
		} else if (abs_input >= static_cast<int32_t>(kInputGateOpenThreshold)) {
			input_gate_open_ = true;
		}
		if (!input_gate_open_) {
			input_sample = 0;
		}
	}

	if (kTestMode == AudioTestMode::kDryPass) {
		dac_.write_channel_a_raw(sample_to_dac_u12(input_sample));
		mark_overrun_if_needed(isr_overrun_count_, tick_start_us, kAudioPeriodUs);
		return true;
	}
	const uint32_t read_index_a = (write_index_ + kMaxDelaySamples - delay_int) % kMaxDelaySamples;
	const uint32_t read_index_b =
		(write_index_ + kMaxDelaySamples - (delay_int + 1)) % kMaxDelaySamples;

	const int32_t delayed_a = delay_buffer_[read_index_a];
	const int32_t delayed_b = delay_buffer_[read_index_b];
	const int32_t delayed_interp = static_cast<int32_t>(
		((static_cast<int64_t>(delayed_a) * static_cast<int64_t>(65536u - frac_q16)) +
			(static_cast<int64_t>(delayed_b) * static_cast<int64_t>(frac_q16))) >>
		16);
	const int16_t delayed_sample = clamp_i16(delayed_interp);

	const int32_t tone_lp = tone_lp_q15_ +
		((static_cast<int32_t>(params.tone_q15) *
			 (static_cast<int32_t>(delayed_sample) - tone_lp_q15_)) >>
			15);
	tone_lp_q15_ = tone_lp;
	const int16_t tone_sample = clamp_i16(tone_lp);

	int16_t write_sample = 0;
	if (params.freeze) {
		write_sample = delayed_sample;
	} else {
		const int32_t feedback_part =
			(static_cast<int32_t>(tone_sample) * static_cast<int32_t>(params.feedback_q15)) >> 15;
		write_sample = soft_clip_i16(static_cast<int32_t>(input_sample) + feedback_part);
	}

	delay_buffer_[write_index_] = write_sample;
	write_index_++;
	if (write_index_ >= kMaxDelaySamples) {
		write_index_ = 0;
	}

	const int32_t dry_part = (static_cast<int32_t>(input_sample) *
							 static_cast<int32_t>(kQ15Max - params.mix_q15)) >>
		15;
	const int32_t wet_part =
		(static_cast<int32_t>(delayed_sample) * static_cast<int32_t>(params.mix_q15)) >> 15;
	const int16_t output_sample = clamp_i16(dry_part + wet_part);

	dac_.write_channel_a_raw(sample_to_dac_u12(output_sample));
	mark_overrun_if_needed(isr_overrun_count_, tick_start_us, kAudioPeriodUs);

	return true;
}

}  // namespace firmware
