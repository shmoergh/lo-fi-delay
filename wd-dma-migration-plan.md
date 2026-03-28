# Working Doc: DMA Migration Plan (`v0.1-poc` -> `v0.2`)

## Goal
Move from the current timer + direct ADC read approach to a cleaner, phased architecture centered on DMA audio input, while keeping each step testable and safe.

## Locked Decisions
- Branch baseline: current working tree
- Migration strategy: incremental, ADC DMA first
- Acceptance style: fixed listening test script + serial counters
- SDK policy: no Brain SDK API changes (firmware-local implementation only)

## Phase 0: Freeze Current PoC
- Create branch `v0.1-poc` from current local state.
- Commit current firmware as baseline.
- Push `v0.1-poc` to `origin`.
- Treat this as the known reference point for all A/B comparisons.

## Phase 1: Add Diagnostics (No Architecture Change)
- Keep audio behavior unchanged.
- Add counters:
  - `audio_tick_count`
  - `adc_stale_sample_count`
  - `control_lock_events`
  - `control_lock_total_us`
  - `control_lock_max_us`
- Keep compile-time audio A/B modes:
  - `normal`
  - `drypass`
  - `dac-midpoint`
- Print compact debug status every 300 ms.

### Exit Criteria
- Existing behavior is preserved.
- Counters are visible and stable.
- No regression in pot stability or distortion.

## Phase 2: Implement ADC DMA Input (`AudioInputDma`)
- Add firmware-local `AudioInputDma` class:
  - `init(target_hz)`
  - `start()`, `stop()`
  - `latest_q15()`
  - `pause_for_control()`, `resume_after_control()`
- Keep timer ISR and delay DSP unchanged.
- Replace direct ISR ADC reads with `latest_q15()`.
- Keep one-pot-per-control-tick logic.

### Exit Criteria
- Audio runs reliably from DMA input.
- Pot reads still stable.
- No increased movement distortion.

## Phase 3: Stabilize ADC DMA / Control Handoff
- Make `resume_after_control()` deterministic:
  - discard required first sample(s) after handoff.
- Add input DC blocker (1-pole high-pass), enabled by default.
- Keep optional input averaging as A/B flag (default off after validation).

### Exit Criteria
- No handoff pops/glitches.
- Idle whine equal or better than previous phase.
- Pot movement distortion equal or better.

## Phase 4 (Optional): SPI DAC DMA Output
- Only do this if Phase 3 is insufficient.
- Add firmware-local `AudioOutputDmaSpi`.
- ISR computes sample and submits to DMA TX path instead of blocking SPI write.

### Exit Criteria
- Measurable/sensible improvement vs Phase 3.
- Complexity justified by audible result.

## Phase 5: Cleanup + RC Preparation
- Remove temporary experimental toggles not needed for release.
- Keep only essential debug and test-mode switches.
- Branch/cut `v0.2-rc1` from the validated phase result.

## Fixed Test Script (Run After Every Phase)
1. `normal`, no input cable, 20s:
   - rate idle whine (`none/low/med/high`), note drift.
2. `drypass`, input A shorted to ground, 20s:
   - rate whine.
3. `normal`, constant tone input:
   - sweep pot1/pot2/pot3 (15s each), rate movement distortion.
4. Serial checks:
   - confirm counters are sane and non-runaway.

## Pass Rule Per Phase
A phase is considered passed only if:
- no pot-stability regression,
- no increase in movement distortion,
- idle whine is same or better (target: one-step improvement by Phase 3),
- tests and logs are documented.

## Commit/Push Rule
After a phase passes:
1. Commit phase work with clear message.
2. Push branch.
3. Tag/log test result summary before starting the next phase.
