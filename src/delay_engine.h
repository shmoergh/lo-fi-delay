#pragma once

#include <cstdint>

#include <pico/time.h>

namespace firmware {

struct DelayParams {
	uint32_t delay_samples;
	int16_t feedback_q15;
	int16_t mix_q15;
	int16_t tone_q15;
	bool freeze;
};

struct DelayStats {
	uint32_t audio_tick_count;
	uint32_t pot_mux_switch_count;
	uint32_t pot_settle_discard_count;
	uint32_t overrun_count;
};

class DelayEngine {
	public:
	enum class AudioTestMode : uint8_t {
		kNormal = 0,
		kDacMidpoint = 1,
		kDryPass = 2
	};

	static const int kAudioPeriodUs = 42;
	static const uint32_t kMaxDelaySamples = 24000;
	static const int16_t kQ15Max = 32767;
	static const int16_t kFeedbackMaxQ15 = 30145;
	static const uint32_t kDelaySlewQ16PerSample = (1u << 12);
	static constexpr AudioTestMode kTestMode = AudioTestMode::kNormal;
	static const int16_t kDcBlockCoeffQ15 = 32604;  // ~0.995

	DelayEngine();

	bool init();
	bool start();
	void stop();
	void clear_and_restart();

	void set_params(const DelayParams& params);
	DelayParams get_params() const;
	uint16_t read_pot_raw_u8(uint8_t pot_index) const;

	DelayStats get_stats() const;
	float sample_rate_hz() const;

	private:
	// Audio callback and DSP path.
	static bool timer_callback(repeating_timer* timer);
	bool process_audio_tick();
	int16_t process_sample(const DelayParams& params, int16_t input_sample, bool delay_slewing);

	// DAC write path (SPI MCP4822 channel A).
	bool init_dac();
	void write_dac_channel_a_raw(uint16_t raw12);

	// ADC+DMA path for audio input and pot-mux control input.
	bool init_adc_dma(float audio_sample_rate_hz);
	bool start_adc_dma();
	void stop_adc_dma();
	void poll_adc_dma();
	uint16_t latest_audio_raw_u12() const;
	void configure_adc_clock(float audio_sample_rate_hz);
	void start_adc_streaming();
	void process_adc_audio_sample(uint16_t raw_u12);
	void process_adc_pot_sample(uint16_t raw_u12);
	void set_active_pot_mux(uint8_t pot_index);

	static DelayEngine* instance_;

	repeating_timer timer_;
	bool running_;

	DelayParams params_;
	volatile uint32_t isr_overrun_count_;
	volatile uint32_t audio_tick_count_;
	volatile uint16_t last_audio_adc_raw_;
	int16_t dc_block_x_prev_;
	int32_t dc_block_y_prev_;
	int16_t prev_input_raw_sample_;
	int16_t prev_delayed_sample_;

	int16_t delay_buffer_[kMaxDelaySamples];
	uint32_t write_index_;
	uint32_t current_delay_q16_;
	int32_t tone_lp_q15_;

	// DAC constants.
	static const uint32_t kDacSpiFrequencyHz = 1000000;

	// ADC DMA constants.
	static const uint8_t kAdcAudioChannel = 1;  // GPIO 27 / ADC1
	static const uint8_t kAdcPotChannel = 0;    // GPIO 26 / ADC0
	static const uint8_t kAdcPotCount = 3;
	static const uint8_t kAdcAudioAverageTaps = 4;
	static const uint16_t kAdcMidRaw = 2048;
	static const uint16_t kAdcMaxRaw = 4095;
	static const uint16_t kAdcPotMaxRaw = 255;
	static const uint16_t kAdcPotSamplesPerHold = 64;
	static const uint8_t kAdcPotDiscardAfterSwitch = 6;
	static const uint32_t kAdcRingSampleCount = 256;
	// Keep ISR time predictable by processing only a bounded number of DMA samples per tick.
	static const uint32_t kAdcMaxSamplesPerPoll = 16;
	static const uint32_t kAdcDmaTransferCount = 0xFFFFFFFFu;
	static const uint32_t kAdcRingBufferBytes = kAdcRingSampleCount * sizeof(uint16_t);
	static const uint8_t kAdcRingWrapBits = 9;  // 2^9 = 512 bytes

	int adc_dma_channel_;
	bool adc_initialized_;
	bool adc_running_;
	uint32_t adc_dma_read_index_;
	bool adc_expect_pot_sample_;

	volatile uint16_t adc_held_audio_raw_u12_;
	volatile uint16_t adc_pot_raw_u12_[kAdcPotCount];
	volatile uint32_t adc_pot_mux_switch_count_;
	volatile uint32_t adc_pot_settle_discard_count_;

	uint8_t adc_active_pot_index_;
	uint8_t adc_pot_discard_remaining_;
	uint32_t adc_pot_accumulator_;
	uint16_t adc_pot_accumulator_count_;

	uint16_t adc_audio_history_[kAdcAudioAverageTaps];
	uint32_t adc_audio_history_sum_;
	uint8_t adc_audio_history_index_;

	alignas(512) volatile uint16_t adc_ring_buffer_[kAdcRingSampleCount];
};

}  // namespace firmware
