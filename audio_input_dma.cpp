#include "audio_input_dma.h"

#include <cstring>

#include <hardware/adc.h>
#include <hardware/dma.h>
#include <hardware/sync.h>

#include "brain-common/brain-gpio-setup.h"

namespace firmware {

namespace {

template <typename T>
T clamp_value(T v, T lo, T hi) {
	if (v < lo) return lo;
	if (v > hi) return hi;
	return v;
}

constexpr uint32_t kRingBufferBytes = 64u * sizeof(uint16_t);
static_assert((kRingBufferBytes & (kRingBufferBytes - 1)) == 0, "Ring buffer bytes must be power of 2");
constexpr uint8_t kRingWrapBits = 7;	// 2^7 = 128 bytes
static_assert((1u << kRingWrapBits) == kRingBufferBytes, "Ring wrap bits must match ring size");

}  // namespace

bool AudioInputDma::init(float target_sample_rate_hz) {
	if (initialized_) {
		return true;
	}

	adc_init();
	adc_gpio_init(GPIO_BRAIN_AUDIO_CV_IN_A);
	adc_select_input(kAudioAdcChannel);
	configure_adc_clock(target_sample_rate_hz);
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
	held_raw_u12_ = 2048;
	settle_discard_remaining_ = kPostResumeDiscardSamples;
	initialized_ = true;
	return true;
}

bool AudioInputDma::start() {
	if (!initialized_) {
		return false;
	}
	if (running_) {
		return true;
	}

	const uint32_t irq_state = save_and_disable_interrupts();
	start_streaming_locked();
	running_ = true;
	paused_ = false;
	settle_discard_remaining_ = kPostResumeDiscardSamples;
	restore_interrupts(irq_state);
	return true;
}

void AudioInputDma::stop() {
	if (!initialized_ || !running_) {
		return;
	}

	const uint32_t irq_state = save_and_disable_interrupts();
	adc_run(false);
	dma_channel_abort(static_cast<uint>(dma_channel_));
	adc_fifo_drain();
	running_ = false;
	paused_ = false;
	restore_interrupts(irq_state);
}

void AudioInputDma::pause_for_control() {
	if (!running_ || paused_) {
		return;
	}

	const uint32_t irq_state = save_and_disable_interrupts();
	held_raw_u12_ = latest_raw_u12();
	adc_run(false);
	dma_channel_abort(static_cast<uint>(dma_channel_));
	adc_fifo_drain();
	paused_ = true;
	restore_interrupts(irq_state);
}

void AudioInputDma::resume_after_control() {
	if (!running_ || !paused_) {
		return;
	}

	const uint32_t irq_state = save_and_disable_interrupts();
	start_streaming_locked();
	paused_ = false;
	settle_discard_remaining_ = kPostResumeDiscardSamples;
	restore_interrupts(irq_state);
}

uint16_t AudioInputDma::latest_raw_u12() {
	if (!running_ || paused_) {
		return held_raw_u12_;
	}

	const uintptr_t base = reinterpret_cast<uintptr_t>(ring_buffer_);
	const uintptr_t write_addr = dma_channel_hw_addr(static_cast<uint>(dma_channel_))->write_addr;
	const uintptr_t byte_offset = (write_addr - base) & (kRingBufferBytes - 1u);
	const uint32_t next_index = static_cast<uint32_t>(byte_offset >> 1);
	const uint32_t last_index = (next_index - 1u) & (kRingSampleCount - 1u);
	uint32_t sum = 0;
	for (uint8_t i = 0; i < kOutputAverageTaps; i++) {
		const uint32_t idx = (last_index - i) & (kRingSampleCount - 1u);
		sum += static_cast<uint16_t>(ring_buffer_[idx] & 0x0FFFu);
	}
	const uint16_t raw = static_cast<uint16_t>(sum / static_cast<uint32_t>(kOutputAverageTaps));

	if (settle_discard_remaining_ > 0) {
		settle_discard_remaining_--;
		return held_raw_u12_;
	}

	held_raw_u12_ = raw;
	return raw;
}

void AudioInputDma::configure_adc_clock(float target_sample_rate_hz) {
	const float safe_rate = clamp_value<float>(target_sample_rate_hz, 1000.0f, 200000.0f);
	const float adc_clock_hz = 48000000.0f;
	const float divisor = (adc_clock_hz / safe_rate) - 1.0f;
	adc_set_clkdiv(divisor);
}

void AudioInputDma::start_streaming_locked() {
	adc_select_input(kAudioAdcChannel);
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

AudioInputDma::AudioInputDma() :
	dma_channel_(-1),
	initialized_(false),
	running_(false),
	paused_(false),
	settle_discard_remaining_(0),
	held_raw_u12_(2048),
	ring_buffer_{} {}

}  // namespace firmware
