#include "delay_engine.h"

#include <hardware/adc.h>
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/spi.h>
#include <hardware/sync.h>
#include <pico/time.h>

#include <cstring>

#include "brain/include/common.h"
#include "brain/include/gpio-setup.h"

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
	(kAudioCvInVoltageAtMinus5V / kAdcVoltageRef) *
	kAdcMaxValue);
const int32_t kInputHighRaw = static_cast<int32_t>(
	(kAudioCvInVoltageAtPlus5V / kAdcVoltageRef) *
	kAdcMaxValue);
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
	audio_tick_count_(0),
	last_audio_adc_raw_(kAdcMidRaw),
	dc_block_x_prev_(0),
	dc_block_y_prev_(0),
	prev_input_raw_sample_(0),
	prev_delayed_sample_(0),
	write_index_(0),
	current_delay_q16_(1u << 16),
	tone_lp_q15_(0),
	adc_dma_channel_(-1),
	adc_initialized_(false),
	adc_running_(false),
	adc_dma_read_index_(0),
	adc_expect_pot_sample_(true),
	adc_held_audio_raw_u12_(kAdcMidRaw),
	adc_pot_raw_u12_{kAdcMidRaw, kAdcMidRaw, kAdcMidRaw},
	adc_pot_mux_switch_count_(0),
	adc_pot_settle_discard_count_(0),
	adc_active_pot_index_(0),
	adc_pot_discard_remaining_(0),
	adc_pot_accumulator_(0),
	adc_pot_accumulator_count_(0),
	adc_audio_history_{kAdcMidRaw, kAdcMidRaw, kAdcMidRaw, kAdcMidRaw},
	adc_audio_history_sum_(static_cast<uint32_t>(kAdcMidRaw) * kAdcAudioAverageTaps),
	adc_audio_history_index_(0),
	adc_ring_buffer_{} {
	params_.delay_samples = static_cast<uint32_t>(0.35f * sample_rate_hz());
	params_.feedback_q15 = static_cast<int16_t>(0.35f * static_cast<float>(kQ15Max));
	params_.mix_q15 = static_cast<int16_t>(0.45f * static_cast<float>(kQ15Max));
	params_.tone_q15 = static_cast<int16_t>(0.22f * static_cast<float>(kQ15Max));
	params_.freeze = false;
	current_delay_q16_ = params_.delay_samples << 16;
}

bool DelayEngine::init() {
	if (!init_dac()) {
		return false;
	}

	memset(delay_buffer_, 0, sizeof(delay_buffer_));
	write_index_ = 0;
	current_delay_q16_ = params_.delay_samples << 16;
	tone_lp_q15_ = 0;
	isr_overrun_count_ = 0;
	audio_tick_count_ = 0;
	dc_block_x_prev_ = 0;
	dc_block_y_prev_ = 0;
	prev_input_raw_sample_ = 0;
	prev_delayed_sample_ = 0;
	last_audio_adc_raw_ = kAdcMidRaw;

	if (!init_adc_dma(sample_rate_hz())) {
		return false;
	}

	instance_ = this;
	return true;
}

bool DelayEngine::start() {
	if (running_) return true;
	if (!start_adc_dma()) {
		return false;
	}
	running_ = add_repeating_timer_us(-kAudioPeriodUs, DelayEngine::timer_callback, nullptr, &timer_);
	if (!running_) {
		stop_adc_dma();
	}
	return running_;
}

void DelayEngine::stop() {
	if (running_) {
		cancel_repeating_timer(&timer_);
		running_ = false;
	}
	stop_adc_dma();
}

void DelayEngine::clear_and_restart() {
	stop();
	memset(delay_buffer_, 0, sizeof(delay_buffer_));
	write_index_ = 0;
	current_delay_q16_ = params_.delay_samples << 16;
	tone_lp_q15_ = 0;
	dc_block_x_prev_ = 0;
	dc_block_y_prev_ = 0;
	prev_input_raw_sample_ = 0;
	prev_delayed_sample_ = 0;
	last_audio_adc_raw_ = kAdcMidRaw;
	start();
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
	const uint16_t raw12 = adc_pot_raw_u12_[pot_index];
	return static_cast<uint16_t>((static_cast<uint32_t>(raw12) * kAdcPotMaxRaw) / kAdcMaxRaw);
}

DelayParams DelayEngine::get_params() const {
	const uint32_t irq_state = save_and_disable_interrupts();
	const DelayParams copy = params_;
	restore_interrupts(irq_state);
	return copy;
}

DelayStats DelayEngine::get_stats() const {
	const uint32_t irq_state = save_and_disable_interrupts();
	DelayStats stats{};
	stats.audio_tick_count = audio_tick_count_;
	stats.pot_mux_switch_count = adc_pot_mux_switch_count_;
	stats.pot_settle_discard_count = adc_pot_settle_discard_count_;
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

// Main audio ISR tick: pull ADC sample, run delay DSP, and write DAC.
bool DelayEngine::process_audio_tick() {
	const uint32_t tick_start_us = time_us_32();
	audio_tick_count_++;

	if (kTestMode == AudioTestMode::kDacMidpoint) {
		write_dac_channel_a_raw(2048);
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
	const bool delay_slewing = (current_delay_q16_ != target_delay_q16);

	poll_adc_dma();
	const uint16_t adc_raw = latest_audio_raw_u12();
	last_audio_adc_raw_ = adc_raw;
	int16_t input_sample = adc_to_audio_sample(adc_raw);

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
		write_dac_channel_a_raw(sample_to_dac_u12(input_sample));
		mark_overrun_if_needed(isr_overrun_count_, tick_start_us, kAudioPeriodUs);
		return true;
	}

	const int16_t output_sample = process_sample(params, input_sample, delay_slewing);
	write_dac_channel_a_raw(sample_to_dac_u12(output_sample));
	mark_overrun_if_needed(isr_overrun_count_, tick_start_us, kAudioPeriodUs);
	return true;
}

// Core delay sample step (fractional read, feedback write, dry/wet output mix).
int16_t DelayEngine::process_sample(
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

// ---- DAC implementation --------------------------------------------------

bool DelayEngine::init_dac() {
	spi_init(spi0, kDacSpiFrequencyHz);
	spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

	gpio_set_function(GPIO_BRAIN_AUDIO_CV_OUT_SCK, GPIO_FUNC_SPI);
	gpio_set_function(GPIO_BRAIN_AUDIO_CV_OUT_TX, GPIO_FUNC_SPI);

	gpio_init(GPIO_BRAIN_AUDIO_CV_OUT_CS);
	gpio_set_dir(GPIO_BRAIN_AUDIO_CV_OUT_CS, GPIO_OUT);
	gpio_put(GPIO_BRAIN_AUDIO_CV_OUT_CS, 1);

	gpio_init(GPIO_BRAIN_AUDIO_CV_OUT_COUPLING_A);
	gpio_set_dir(GPIO_BRAIN_AUDIO_CV_OUT_COUPLING_A, GPIO_OUT);
	gpio_put(GPIO_BRAIN_AUDIO_CV_OUT_COUPLING_A, 1);  // AC coupled

	gpio_init(GPIO_BRAIN_AUDIO_CV_OUT_COUPLING_B);
	gpio_set_dir(GPIO_BRAIN_AUDIO_CV_OUT_COUPLING_B, GPIO_OUT);
	gpio_put(GPIO_BRAIN_AUDIO_CV_OUT_COUPLING_B, 0);  // Keep channel B DC coupled

	return true;
}

void DelayEngine::write_dac_channel_a_raw(uint16_t raw12) {
	raw12 = clamp_value<uint16_t>(raw12, 0, 4095);

	constexpr uint8_t kConfig = (0u << 3) | (0u << 2) | (0u << 1) | 1u;
	uint8_t data[2];
	data[0] = static_cast<uint8_t>((kConfig << 4) | ((raw12 >> 8) & 0x0F));
	data[1] = static_cast<uint8_t>(raw12 & 0xFF);

	asm volatile("nop \n nop \n nop");
	gpio_put(GPIO_BRAIN_AUDIO_CV_OUT_CS, 0);
	asm volatile("nop \n nop \n nop");
	spi_write_blocking(spi0, data, 2);
	asm volatile("nop \n nop \n nop");
	gpio_put(GPIO_BRAIN_AUDIO_CV_OUT_CS, 1);
	asm volatile("nop \n nop \n nop");
}

// ---- ADC DMA implementation ---------------------------------------------

bool DelayEngine::init_adc_dma(float audio_sample_rate_hz) {
	if (adc_initialized_) {
		return true;
	}

	adc_init();
	adc_gpio_init(GPIO_BRAIN_AUDIO_CV_IN_A);
	adc_gpio_init(GPIO_BRAIN_POTMUX_ADC);

	gpio_init(GPIO_BRAIN_POTMUX_S0);
	gpio_set_dir(GPIO_BRAIN_POTMUX_S0, GPIO_OUT);
	gpio_init(GPIO_BRAIN_POTMUX_S1);
	gpio_set_dir(GPIO_BRAIN_POTMUX_S1, GPIO_OUT);
	set_active_pot_mux(0);

	configure_adc_clock(audio_sample_rate_hz);
	adc_fifo_setup(
		true,   // Write each conversion to FIFO
		true,   // Enable DREQ for DMA
		1,      // DREQ when at least one sample is available
		false,  // No error bit in FIFO stream
		false   // Keep full 12-bit samples
	);
	adc_fifo_drain();

	adc_dma_channel_ = dma_claim_unused_channel(true);
	memset((void*) adc_ring_buffer_, 0, sizeof(adc_ring_buffer_));
	adc_initialized_ = true;
	return true;
}

bool DelayEngine::start_adc_dma() {
	if (!adc_initialized_) {
		return false;
	}
	if (adc_running_) {
		return true;
	}

	const uint32_t irq_state = save_and_disable_interrupts();
	adc_dma_read_index_ = 0;
	adc_expect_pot_sample_ = true;
	adc_held_audio_raw_u12_ = kAdcMidRaw;
	adc_pot_raw_u12_[0] = kAdcMidRaw;
	adc_pot_raw_u12_[1] = kAdcMidRaw;
	adc_pot_raw_u12_[2] = kAdcMidRaw;
	adc_pot_mux_switch_count_ = 0;
	adc_pot_settle_discard_count_ = 0;
	adc_active_pot_index_ = 0;
	adc_pot_discard_remaining_ = kAdcPotDiscardAfterSwitch;
	adc_pot_accumulator_ = 0;
	adc_pot_accumulator_count_ = 0;
	adc_audio_history_sum_ = 0;
	for (uint8_t i = 0; i < kAdcAudioAverageTaps; i++) {
		adc_audio_history_[i] = kAdcMidRaw;
		adc_audio_history_sum_ += kAdcMidRaw;
	}
	adc_audio_history_index_ = 0;

	start_adc_streaming();
	adc_running_ = true;
	restore_interrupts(irq_state);
	return true;
}

void DelayEngine::stop_adc_dma() {
	if (!adc_initialized_ || !adc_running_) {
		return;
	}

	const uint32_t irq_state = save_and_disable_interrupts();
	adc_run(false);
	dma_channel_abort(static_cast<uint>(adc_dma_channel_));
	adc_fifo_drain();
	adc_running_ = false;
	restore_interrupts(irq_state);
}

void DelayEngine::poll_adc_dma() {
	if (!adc_running_) {
		return;
	}

	const uintptr_t base = reinterpret_cast<uintptr_t>(adc_ring_buffer_);
	const uintptr_t write_addr = dma_channel_hw_addr(static_cast<uint>(adc_dma_channel_))->write_addr;
	const uintptr_t byte_offset = (write_addr - base) & (kAdcRingBufferBytes - 1u);
	const uint32_t next_index = static_cast<uint32_t>(byte_offset >> 1);

	uint32_t processed_samples = 0;
	while (adc_dma_read_index_ != next_index) {
		const uint16_t raw_u12 = static_cast<uint16_t>(adc_ring_buffer_[adc_dma_read_index_] & 0x0FFFu);
		if (adc_expect_pot_sample_) {
			process_adc_pot_sample(raw_u12);
		} else {
			process_adc_audio_sample(raw_u12);
		}
		adc_expect_pot_sample_ = !adc_expect_pot_sample_;
		adc_dma_read_index_ = (adc_dma_read_index_ + 1u) & (kAdcRingSampleCount - 1u);
		processed_samples++;
		if (processed_samples >= kAdcMaxSamplesPerPoll) {
			break;
		}
	}
}

uint16_t DelayEngine::latest_audio_raw_u12() const {
	return adc_held_audio_raw_u12_;
}

void DelayEngine::configure_adc_clock(float audio_sample_rate_hz) {
	const float safe_audio_rate = clamp_value<float>(audio_sample_rate_hz, 1000.0f, 48000.0f);
	const float total_conversion_rate_hz = safe_audio_rate * 8.0f;
	const float adc_clock_hz = 48000000.0f;
	const float divisor = (adc_clock_hz / total_conversion_rate_hz) - 1.0f;
	adc_set_clkdiv(divisor);
}

void DelayEngine::start_adc_streaming() {
	set_active_pot_mux(0);
	adc_set_round_robin((1u << kAdcPotChannel) | (1u << kAdcAudioChannel));
	adc_select_input(kAdcPotChannel);
	adc_fifo_drain();
	adc_run(false);

	dma_channel_config cfg = dma_channel_get_default_config(static_cast<uint>(adc_dma_channel_));
	channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
	channel_config_set_read_increment(&cfg, false);
	channel_config_set_write_increment(&cfg, true);
	channel_config_set_dreq(&cfg, DREQ_ADC);
	channel_config_set_ring(&cfg, true, kAdcRingWrapBits);

	dma_channel_configure(
		static_cast<uint>(adc_dma_channel_),
		&cfg,
		(void*) adc_ring_buffer_,
		&adc_hw->fifo,
		kAdcDmaTransferCount,
		false);

	adc_run(true);
	dma_channel_start(static_cast<uint>(adc_dma_channel_));
}

void DelayEngine::process_adc_audio_sample(uint16_t raw_u12) {
	adc_audio_history_sum_ -= adc_audio_history_[adc_audio_history_index_];
	adc_audio_history_[adc_audio_history_index_] = raw_u12;
	adc_audio_history_sum_ += raw_u12;
	adc_audio_history_index_ = static_cast<uint8_t>((adc_audio_history_index_ + 1u) % kAdcAudioAverageTaps);
	adc_held_audio_raw_u12_ = static_cast<uint16_t>(adc_audio_history_sum_ / kAdcAudioAverageTaps);
}

void DelayEngine::process_adc_pot_sample(uint16_t raw_u12) {
	if (adc_pot_discard_remaining_ > 0) {
		adc_pot_discard_remaining_--;
		adc_pot_settle_discard_count_++;
		return;
	}

	adc_pot_accumulator_ += raw_u12;
	adc_pot_accumulator_count_++;
	if (adc_pot_accumulator_count_ < kAdcPotSamplesPerHold) {
		return;
	}

	adc_pot_raw_u12_[adc_active_pot_index_] =
		static_cast<uint16_t>(adc_pot_accumulator_ / static_cast<uint32_t>(adc_pot_accumulator_count_));
	adc_pot_accumulator_ = 0;
	adc_pot_accumulator_count_ = 0;

	const uint8_t next_pot = static_cast<uint8_t>((adc_active_pot_index_ + 1u) % kAdcPotCount);
	set_active_pot_mux(next_pot);
	adc_pot_discard_remaining_ = kAdcPotDiscardAfterSwitch;
	adc_pot_mux_switch_count_++;
}

void DelayEngine::set_active_pot_mux(uint8_t pot_index) {
	adc_active_pot_index_ = static_cast<uint8_t>(pot_index % kAdcPotCount);
	gpio_put(GPIO_BRAIN_POTMUX_S0, adc_active_pot_index_ & 0x01u);
	gpio_put(GPIO_BRAIN_POTMUX_S1, (adc_active_pot_index_ >> 1) & 0x01u);
}

}  // namespace firmware
