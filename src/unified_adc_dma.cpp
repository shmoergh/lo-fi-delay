#include "unified_adc_dma.h"

#include <cstring>

#include <hardware/adc.h>
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/sync.h>

#include "brain/include/gpio-setup.h"

namespace firmware {

namespace {

template <typename T>
T clamp_value(T v, T lo, T hi) {
	if (v < lo) return lo;
	if (v > hi) return hi;
	return v;
}

}  // namespace

UnifiedAdcDma::UnifiedAdcDma() :
	dma_channel_(-1),
	initialized_(false),
	running_(false),
	dma_read_index_(0),
	expect_pot_sample_(true),
	held_audio_raw_u12_(kAudioMidRaw),
	pot_raw_u12_{kAudioMidRaw, kAudioMidRaw, kAudioMidRaw},
	pot_mux_switch_count_(0),
	pot_settle_discard_count_(0),
	active_pot_index_(0),
	pot_discard_remaining_(0),
	pot_accumulator_(0),
	pot_accumulator_count_(0),
	audio_history_{kAudioMidRaw, kAudioMidRaw, kAudioMidRaw, kAudioMidRaw},
	audio_history_sum_(static_cast<uint32_t>(kAudioMidRaw) * kAudioAverageTaps),
	audio_history_index_(0),
	ring_buffer_{} {}

bool UnifiedAdcDma::init(float audio_sample_rate_hz) {
	if (initialized_) {
		return true;
	}

	adc_init();
	adc_gpio_init(GPIO_BRAIN_AUDIO_CV_IN_A);
	adc_gpio_init(GPIO_BRAIN_POTMUX_ADC);

	gpio_init(GPIO_BRAIN_POTMUX_S0);
	gpio_set_dir(GPIO_BRAIN_POTMUX_S0, GPIO_OUT);
	gpio_init(GPIO_BRAIN_POTMUX_S1);
	gpio_set_dir(GPIO_BRAIN_POTMUX_S1, GPIO_OUT);
	set_active_pot_mux_locked(0);

	configure_adc_clock(audio_sample_rate_hz);
	adc_fifo_setup(
		true,   // Write each conversion to FIFO
		true,   // Enable DREQ for DMA
		1,      // DREQ when at least one sample is available
		false,  // No error bit in FIFO stream
		false   // Keep full 12-bit samples
	);
	adc_fifo_drain();

	dma_channel_ = dma_claim_unused_channel(true);
	memset((void*) ring_buffer_, 0, sizeof(ring_buffer_));
	initialized_ = true;
	return true;
}

bool UnifiedAdcDma::start() {
	if (!initialized_) {
		return false;
	}
	if (running_) {
		return true;
	}

	const uint32_t irq_state = save_and_disable_interrupts();
	dma_read_index_ = 0;
	expect_pot_sample_ = true;
	held_audio_raw_u12_ = kAudioMidRaw;
	pot_raw_u12_[0] = kAudioMidRaw;
	pot_raw_u12_[1] = kAudioMidRaw;
	pot_raw_u12_[2] = kAudioMidRaw;
	pot_mux_switch_count_ = 0;
	pot_settle_discard_count_ = 0;
	active_pot_index_ = 0;
	pot_discard_remaining_ = kPotDiscardAfterSwitch;
	pot_accumulator_ = 0;
	pot_accumulator_count_ = 0;
	audio_history_sum_ = 0;
	for (uint8_t i = 0; i < kAudioAverageTaps; i++) {
		audio_history_[i] = kAudioMidRaw;
		audio_history_sum_ += kAudioMidRaw;
	}
	audio_history_index_ = 0;

	start_streaming_locked();
	running_ = true;
	restore_interrupts(irq_state);
	return true;
}

void UnifiedAdcDma::stop() {
	if (!initialized_ || !running_) {
		return;
	}

	const uint32_t irq_state = save_and_disable_interrupts();
	adc_run(false);
	dma_channel_abort(static_cast<uint>(dma_channel_));
	adc_fifo_drain();
	running_ = false;
	restore_interrupts(irq_state);
}

void UnifiedAdcDma::poll() {
	if (!running_) {
		return;
	}

	const uintptr_t base = reinterpret_cast<uintptr_t>(ring_buffer_);
	const uintptr_t write_addr = dma_channel_hw_addr(static_cast<uint>(dma_channel_))->write_addr;
	const uintptr_t byte_offset = (write_addr - base) & (kRingBufferBytes - 1u);
	const uint32_t next_index = static_cast<uint32_t>(byte_offset >> 1);

	while (dma_read_index_ != next_index) {
		const uint16_t raw_u12 = static_cast<uint16_t>(ring_buffer_[dma_read_index_] & 0x0FFFu);
		if (expect_pot_sample_) {
			process_pot_sample_locked(raw_u12);
		} else {
			process_audio_sample_locked(raw_u12);
		}
		expect_pot_sample_ = !expect_pot_sample_;
		dma_read_index_ = (dma_read_index_ + 1u) & (kRingSampleCount - 1u);
	}
}

uint16_t UnifiedAdcDma::latest_audio_raw_u12() const {
	return held_audio_raw_u12_;
}

uint16_t UnifiedAdcDma::pot_raw_u8(uint8_t pot_index) const {
	if (pot_index >= kPotCount) {
		return 0;
	}
	const uint16_t raw12 = pot_raw_u12_[pot_index];
	return static_cast<uint16_t>((static_cast<uint32_t>(raw12) * kPotMaxRaw) / kAdcMaxRaw);
}

UnifiedAdcStats UnifiedAdcDma::get_stats() const {
	UnifiedAdcStats stats{};
	stats.pot_mux_switch_count = pot_mux_switch_count_;
	stats.pot_settle_discard_count = pot_settle_discard_count_;
	return stats;
}

void UnifiedAdcDma::configure_adc_clock(float audio_sample_rate_hz) {
	const float safe_audio_rate = clamp_value<float>(audio_sample_rate_hz, 1000.0f, 48000.0f);
	const float total_conversion_rate_hz = safe_audio_rate * 8.0f;
	const float adc_clock_hz = 48000000.0f;
	const float divisor = (adc_clock_hz / total_conversion_rate_hz) - 1.0f;
	adc_set_clkdiv(divisor);
}

void UnifiedAdcDma::start_streaming_locked() {
	set_active_pot_mux_locked(0);
	adc_set_round_robin((1u << kPotAdcChannel) | (1u << kAudioAdcChannel));
	adc_select_input(kPotAdcChannel);
	adc_fifo_drain();
	adc_run(false);

	dma_channel_config cfg = dma_channel_get_default_config(static_cast<uint>(dma_channel_));
	channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
	channel_config_set_read_increment(&cfg, false);
	channel_config_set_write_increment(&cfg, true);
	channel_config_set_dreq(&cfg, DREQ_ADC);
	channel_config_set_ring(&cfg, true, kRingWrapBits);

	dma_channel_configure(
		static_cast<uint>(dma_channel_),
		&cfg,
		(void*) ring_buffer_,
		&adc_hw->fifo,
		kDmaTransferCount,
		false);

	adc_run(true);
	dma_channel_start(static_cast<uint>(dma_channel_));
}

void UnifiedAdcDma::process_audio_sample_locked(uint16_t raw_u12) {
	audio_history_sum_ -= audio_history_[audio_history_index_];
	audio_history_[audio_history_index_] = raw_u12;
	audio_history_sum_ += raw_u12;
	audio_history_index_ = static_cast<uint8_t>((audio_history_index_ + 1u) % kAudioAverageTaps);
	held_audio_raw_u12_ = static_cast<uint16_t>(audio_history_sum_ / kAudioAverageTaps);
}

void UnifiedAdcDma::process_pot_sample_locked(uint16_t raw_u12) {
	if (pot_discard_remaining_ > 0) {
		pot_discard_remaining_--;
		pot_settle_discard_count_++;
		return;
	}

	pot_accumulator_ += raw_u12;
	pot_accumulator_count_++;
	if (pot_accumulator_count_ < kPotSamplesPerHold) {
		return;
	}

	pot_raw_u12_[active_pot_index_] =
		static_cast<uint16_t>(pot_accumulator_ / static_cast<uint32_t>(pot_accumulator_count_));
	pot_accumulator_ = 0;
	pot_accumulator_count_ = 0;

	const uint8_t next_pot = static_cast<uint8_t>((active_pot_index_ + 1u) % kPotCount);
	set_active_pot_mux_locked(next_pot);
	pot_discard_remaining_ = kPotDiscardAfterSwitch;
	pot_mux_switch_count_++;
}

void UnifiedAdcDma::set_active_pot_mux_locked(uint8_t pot_index) {
	active_pot_index_ = static_cast<uint8_t>(pot_index % kPotCount);
	gpio_put(GPIO_BRAIN_POTMUX_S0, active_pot_index_ & 0x01u);
	gpio_put(GPIO_BRAIN_POTMUX_S1, (active_pot_index_ >> 1) & 0x01u);
}

}  // namespace firmware
