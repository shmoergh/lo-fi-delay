# Lo-Fi Delay Firmware (Brain)

Digital mono delay firmware for the Shmoergh Brain platform, optimized for stable interrupt-driven audio with DMA-based ADC input.

Current release target: **Pico 2 only** (`rp2350-arm-s`).
SDK baseline: **Brain SDK 2.0** (UI/control via `Brain` wrapper).

## What This Firmware Does

- Processes `Audio/CV In A` and outputs on `Audio/CV Out A`.
- Uses a fixed-rate audio interrupt for output timing.
- Uses DMA to continuously sample ADC input and reduce control-read interference.
- Provides three core delay controls:
- `Time` (log scale)
- `Feedback`
- `Mix` (wet/dry)
- Supports performance controls:
- `Button A`: tap tempo (single taps), clear delay buffer (long press)
- `Button B`: freeze (momentary hold)
- Drives 6 panel LEDs with an organic animation.
- Uses button LED for tempo pulse / freeze indication.

## Control Map

- `Pot 1`: delay time, log-mapped from `30 ms` to `1000 ms`.
- `Pot 2`: feedback amount, clamped to max `0.92`.
- `Pot 3`: wet/dry mix (`0.0` dry to `1.0` wet).
- `Button A` short taps: tap tempo.
- `Button A` long press (`~500 ms`): clear delay buffer.
- `Button B` hold: freeze delay write path (loop hold); release resumes normal recording.

Tap tempo uses pickup behavior:
- after tap tempo is set, the time pot takes control again only after moving enough from the captured position.

## LED Behavior

- 6 panel LEDs: continuous random-fluid PWM animation (not parameter meter bars).
- Button LED:
- freeze held: solid on
- otherwise: pulses at current delay tempo rate

## Audio Architecture

### Timing Model

- Audio ISR cadence: `42 us` period (`~23.81 kHz`).
- DAC writes happen in ISR only.
- Main loop never writes audio samples.

### Input Path

- Unified ADC DMA round-robin samples ADC0 (pot mux) and ADC1 (audio input) continuously.
- ISR reads latest smoothed audio sample from this unified DMA stream.
- Pot mux switching, settle discard, and pot averaging are handled inside the unified ADC subsystem.

### DSP Path (ISR)

- Convert ADC raw -> centered signed audio sample.
- Apply 1-pole DC blocker (always on).
- Apply short input averaging (always on).
- Interpolated delay read (fractional delay using Q16 interpolation).
- Feedback tone shaping via 1-pole low-pass in loop.
- Soft-clip feedback write to avoid hard digital clipping.
- Dry/wet mix in fixed-point.
- Quantize and write 12-bit DAC sample.

### Delay Memory

- Delay buffer: static `int16_t delay_buffer_[24000]`.
- Max delay around `1.0 s` at current sample rate.
- No dynamic allocation in audio engine.

## Platform Support

- Current release build target: `pico2` (`rp2350-arm-s`) only.
- RP2040 (`pico`) build is intentionally disabled in `build-firmware.sh`.

## Build

### Prerequisites

- CMake 3.22+
- Arm GCC toolchain compatible with Pico SDK
- `PICO_SDK_PATH` set in your environment
- Git submodules initialized

### Build Command

```bash
./build-firmware.sh
```

This builds the Pico 2 target and copies UF2 artifact to repo root:

- `lo-fi-delay-pico-2.uf2`

```bash
cmake -S . -B build -DPICO_BOARD=pico2 -DPICO_PLATFORM=rp2350-arm-s
cmake --build build
```

## Flash

1. Hold BOOTSEL on target hardware while connecting USB.
2. Mount the drive.
3. Copy `lo-fi-delay-pico-2.uf2` to the mounted drive.

## Logging and Debug Modes

### Runtime Logging

- Logging is disabled for release by default:
- `DelayApp::kEnableLogging = false` in `src/delay_app.h`.
- To enable serial status logging, set it to `true` and rebuild.

### Audio Test Modes

Compile-time mode switch in `src/delay_engine.h`:
- `DelayEngine::kTestMode = AudioTestMode::kNormal`
- Alternate modes available:
- `kDryPass`
- `kDacMidpoint`

## Key Tuning Constants

Useful constants for sound/behavior tuning:

- `DelayEngine::kAudioPeriodUs = 42`
- `DelayEngine::kMaxDelaySamples = 24000`
- `DelayEngine::kFeedbackMaxQ15 = 30145` (`~0.92`)
- `DelayApp::kMinDelayMs = 30.0`
- `DelayApp::kMaxDelayMs = 1000.0`
- `DelayApp::kDelaySmoothingAlpha = 0.08`
- `DelayApp::kPotDeadbandTime = 2`
- `DelayApp::kPotDeadbandFeedback = 1`
- `DelayApp::kPotDeadbandMix = 1`

DMA input specifics:

- Unified ADC DMA ring size: `256` samples
- Audio averaging taps: `4` samples
- Pot settle discard after mux switch: `6` samples

## Repository Structure

- `main.cpp`: entrypoint
- `src/delay_app.h/.cpp`: UI, controls, LEDs, parameter mapping, app loop
- `src/delay_engine.h/.cpp`: ISR audio engine + DSP + delay line
- `src/unified_adc_dma.h/.cpp`: unified ADC DMA (audio + pot-mux scheduler)
- `src/fast_dac_out.h/.cpp`: fast SPI DAC writes and output coupling setup
- `brain-sdk/`: Brain SDK submodule

## Known Scope and Limitations (Current Release)

- Mono only (`In A` -> `Out A`)
- Max delay about 1 second
- No external clock sync
- No preset memory
- No MIDI
- No stereo processing

## Updating Brain SDK Submodule

```bash
cd brain-sdk
git checkout 2.0
git pull origin 2.0
cd ..
git add brain-sdk
git commit -m "Update brain-sdk"
```
