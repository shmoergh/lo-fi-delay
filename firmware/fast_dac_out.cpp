#include "fast_dac_out.h"

#include <hardware/gpio.h>
#include <hardware/spi.h>

#include "brain-common/brain-gpio-setup.h"

namespace firmware {

namespace {

template <typename T>
T clamp_value(T v, T lo, T hi) {
	if (v < lo) return lo;
	if (v > hi) return hi;
	return v;
}

}  // namespace

bool FastDacOut::init() {
	spi_init(spi0, kSpiFrequencyHz);
	spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

	gpio_set_function(GPIO_BRAIN_AUDIO_CV_OUT_SCK, GPIO_FUNC_SPI);
	gpio_set_function(GPIO_BRAIN_AUDIO_CV_OUT_TX, GPIO_FUNC_SPI);

	gpio_init(GPIO_BRAIN_AUDIO_CV_OUT_CS);
	gpio_set_dir(GPIO_BRAIN_AUDIO_CV_OUT_CS, GPIO_OUT);
	gpio_put(GPIO_BRAIN_AUDIO_CV_OUT_CS, 1);

	gpio_init(GPIO_BRAIN_AUDIO_CV_OUT_COUPLING_A);
	gpio_set_dir(GPIO_BRAIN_AUDIO_CV_OUT_COUPLING_A, GPIO_OUT);
	gpio_put(GPIO_BRAIN_AUDIO_CV_OUT_COUPLING_A, 1);	// AC coupled

	gpio_init(GPIO_BRAIN_AUDIO_CV_OUT_COUPLING_B);
	gpio_set_dir(GPIO_BRAIN_AUDIO_CV_OUT_COUPLING_B, GPIO_OUT);
	gpio_put(GPIO_BRAIN_AUDIO_CV_OUT_COUPLING_B, 0);	// Keep channel B DC coupled

	return true;
}

void FastDacOut::write_channel_a_raw(uint16_t raw12) {
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

}  // namespace firmware
