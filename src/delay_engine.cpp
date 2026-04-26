#include "delay_engine.h"

#include <hardware/sync.h>

#include <cstdio>
#include <cstring>

#include "brain/include/common.h"

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
	(kAudioCvInVoltageAtMinus5V / kAdcVoltageRef) *
	kAdcMaxValue);
const int32_t kInputHighRaw = static_cast<int32_t>(
	(kAudioCvInVoltageAtPlus5V / kAdcVoltageRef) *
	kAdcMaxValue);
const int32_t kInputMidRaw = (kInputLowRaw + kInputHighRaw) / 2;
const int32_t kInputHalfSpanRaw = ((kInputHighRaw - kInputLowRaw) > 0)
	? ((kInputHighRaw - kInputLowRaw) / 2)
	: 1;

int16_t normalize_audio_input_sample(int16_t input_sample) {
	// AudioProcessor provides an ADC-centered int16 sample; normalize to board-calibrated -5..+5V span.
	const int32_t adc_raw = 2048 + (static_cast<int32_t>(input_sample) >> 4);
	const int32_t centered = adc_raw - kInputMidRaw;
	const int32_t scaled = (centered * 32767) / kInputHalfSpanRaw;
	return clamp_i16(scaled);
}

}  // namespace

DelayEngine::DelayEngine() :
	brain_(nullptr),
	initialized_(false),
	running_(false),
	dc_block_x_prev_(0),
	dc_block_y_prev_(0),
	prev_input_raw_sample_(0),
	prev_delayed_sample_(0),
	write_index_(0),
	current_delay_q16_(1u << 16),
	tone_lp_q15_(0) {
	params_.delay_samples = static_cast<uint32_t>(0.35f * sample_rate_hz());
	params_.feedback_q15 = static_cast<int16_t>(0.35f * static_cast<float>(kQ15Max));
	params_.mix_q15 = static_cast<int16_t>(0.45f * static_cast<float>(kQ15Max));
	params_.tone_q15 = static_cast<int16_t>(0.22f * static_cast<float>(kQ15Max));
	params_.freeze = false;
	current_delay_q16_ = params_.delay_samples << 16;
	std::memset(delay_buffer_, 0, sizeof(delay_buffer_));
}

bool DelayEngine::init(Brain& brain) {
	if (initialized_) {
		return true;
	}

	brain_ = &brain;
	clear_and_restart();
	initialized_ = true;
	return true;
}

bool DelayEngine::start() {
	if (!initialized_ || brain_ == nullptr) {
		return false;
	}
	if (running_) {
		return true;
	}

	AudioProcessorConfig config{};
	config.sample_period_us = static_cast<uint32_t>(kAudioPeriodUs);

	if (try_start_audio_processor(config)) {
		running_ = true;
		return true;
	}

	// Fallback: slightly longer tick to reduce ISR load if the primary period fails.
	AudioProcessorConfig fallback_slow = config;
	fallback_slow.sample_period_us = 50;
	if (try_start_audio_processor(fallback_slow)) {
		running_ = true;
		return true;
	}

	return false;
}

void DelayEngine::stop() {
	if (!running_) {
		return;
	}

	if (brain_ != nullptr && brain_->is_audio_processor_initialized()) {
		brain_->audio_processor.stop();
	}
	running_ = false;
}

void DelayEngine::clear_and_restart() {
	const uint32_t irq_state = save_and_disable_interrupts();
	std::memset(delay_buffer_, 0, sizeof(delay_buffer_));
	write_index_ = 0;
	current_delay_q16_ = params_.delay_samples << 16;
	tone_lp_q15_ = 0;
	dc_block_x_prev_ = 0;
	dc_block_y_prev_ = 0;
	prev_input_raw_sample_ = 0;
	prev_delayed_sample_ = 0;
	restore_interrupts(irq_state);
}

void DelayEngine::set_params(const DelayParams& params) {
	const uint32_t irq_state = save_and_disable_interrupts();
	params_ = params;
	restore_interrupts(irq_state);
}

uint16_t DelayEngine::read_pot_raw_u8(uint8_t pot_index) const {
	if (pot_index >= kAdcPotCount) {
		return 0;
	}
	if (brain_ == nullptr || !brain_->is_audio_processor_initialized()) {
		return kAdcPotMaxRaw / 2;
	}
	return brain_->audio_processor.get_pot_raw_u8(pot_index);
}

DelayParams DelayEngine::get_params() const {
	const uint32_t irq_state = save_and_disable_interrupts();
	const DelayParams copy = params_;
	restore_interrupts(irq_state);
	return copy;
}

DelayStats DelayEngine::get_stats() const {
	DelayStats stats{};
	if (brain_ == nullptr || !brain_->is_audio_processor_initialized()) {
		return stats;
	}

	const AudioProcessorStats audio_stats = brain_->audio_processor.get_stats();
	stats.audio_tick_count = static_cast<uint32_t>(
		(audio_stats.tick_count > 0xFFFFFFFFULL) ? 0xFFFFFFFFULL : audio_stats.tick_count);
	stats.pot_mux_switch_count = audio_stats.pot_mux_switch_count;
	stats.pot_settle_discard_count = audio_stats.pot_settle_discard_count;
	stats.overrun_count = audio_stats.overrun_count;
	return stats;
}

float DelayEngine::sample_rate_hz() const {
	return 1000000.0f / static_cast<float>(kAudioPeriodUs);
}

bool DelayEngine::try_start_audio_processor(const AudioProcessorConfig& config) {
	if (brain_ == nullptr) {
		return false;
	}

	const BrainInitStatus init_status =
		brain_->init_audio_processor(config, &DelayEngine::audio_callback, this);
	if (brain_init_succeeded(init_status) || init_status == BrainInitStatus::kAlreadyInitialized) {
		return true;
	}

	// Diagnostic fallback: call utility directly to distinguish Brain guardrail failures
	// from low-level AudioProcessor initialization failures.
	fprintf(
		stderr,
		"[delay] brain.init_audio_processor failed (period=%lu). Retrying direct init.\n",
		static_cast<unsigned long>(config.sample_period_us));

	const BrainInitStatus direct_status =
		brain_->audio_processor.init(config, &DelayEngine::audio_callback, this);
	if (brain_init_succeeded(direct_status) || direct_status == BrainInitStatus::kAlreadyInitialized) {
		fprintf(stderr, "[delay] direct audio_processor.init succeeded.\n");
		return true;
	}

	fprintf(stderr, "[delay] direct audio_processor.init failed.\n");
	return false;
}

int16_t DelayEngine::audio_callback(
	int16_t input_sample,
	const AudioProcessorFrame* frame,
	void* user_ctx) {
	DelayEngine* engine = static_cast<DelayEngine*>(user_ctx);
	if (engine == nullptr) {
		return input_sample;
	}
	return engine->process_audio_sample(input_sample, frame);
}

int16_t DelayEngine::process_audio_sample(int16_t input_sample, const AudioProcessorFrame* frame) {
	(void) frame;

	if (kTestMode == AudioTestMode::kDacMidpoint) {
		return 0;
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
	const bool delay_slewing = (current_delay_q16_ != target_delay_q16);

	input_sample = normalize_audio_input_sample(input_sample);

	// Remove DC and slow offset drift from the input path.
	const int32_t x = static_cast<int32_t>(input_sample);
	const int32_t y = (x - static_cast<int32_t>(dc_block_x_prev_)) +
		((static_cast<int32_t>(kDcBlockCoeffQ15) * dc_block_y_prev_) >> 15);
	dc_block_x_prev_ = input_sample;
	dc_block_y_prev_ = clamp_value<int32_t>(y, -32768, 32767);
	input_sample = static_cast<int16_t>(dc_block_y_prev_);

	// Light pre-average on input to reduce pointy transients.
	const int32_t averaged =
		(static_cast<int32_t>(input_sample) + static_cast<int32_t>(prev_input_raw_sample_)) / 2;
	prev_input_raw_sample_ = input_sample;
	input_sample = clamp_i16(averaged);

	if (kTestMode == AudioTestMode::kDryPass) {
		return input_sample;
	}

	return process_delay_sample(params, input_sample, delay_slewing);
}

int16_t DelayEngine::process_delay_sample(
	const DelayParams& params,
	int16_t input_sample,
	bool delay_slewing) {
	const uint32_t delay_int = clamp_value<uint32_t>(current_delay_q16_ >> 16, 1, kMaxDelaySamples - 2);
	const uint32_t frac_q16 = current_delay_q16_ & 0xFFFFu;

	const uint32_t read_index_a = (write_index_ + kMaxDelaySamples - delay_int) % kMaxDelaySamples;
	const uint32_t read_index_b =
		(write_index_ + kMaxDelaySamples - (delay_int + 1)) % kMaxDelaySamples;

	const int32_t delayed_a = delay_buffer_[read_index_a];
	const int32_t delayed_b = delay_buffer_[read_index_b];
	const int32_t delayed_interp = static_cast<int32_t>(
		((static_cast<int64_t>(delayed_a) * static_cast<int64_t>(65536u - frac_q16)) +
			(static_cast<int64_t>(delayed_b) * static_cast<int64_t>(frac_q16))) >>
		16);
	int16_t delayed_sample = clamp_i16(delayed_interp);
	if (delay_slewing) {
		delayed_sample = clamp_i16(
			(static_cast<int32_t>(delayed_sample) + static_cast<int32_t>(prev_delayed_sample_)) / 2);
	}
	prev_delayed_sample_ = delayed_sample;

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

	const int32_t dry_part =
		(static_cast<int32_t>(input_sample) * static_cast<int32_t>(kQ15Max - params.mix_q15)) >> 15;
	const int32_t wet_part =
		(static_cast<int32_t>(delayed_sample) * static_cast<int32_t>(params.mix_q15)) >> 15;
	return clamp_i16(dry_part + wet_part);
}

}  // namespace firmware
