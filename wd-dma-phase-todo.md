# DMA Migration Phase Todo

## Phase 0: Baseline Freeze (`v0.1-poc`)
- [x] Create branch `v0.1-poc` from current local state
- [x] Commit current PoC baseline
- [x] Push branch to `origin`
- [ ] Record baseline listening notes (idle whine + pot movement behavior)

## Phase 1: Diagnostics Only (No Audio Architecture Change)
- [x] Add counters (`audio_tick_count`, `adc_stale_sample_count`, `control_lock_events`, `control_lock_total_us`, `control_lock_max_us`)
- [x] Add/keep compact serial print at fixed interval
- [x] Keep test modes (`normal`, `drypass`, `dac-midpoint`)
- [x] Build `pico` and `pico2`
- [x] Run fixed test script
- [x] Compare with baseline and confirm no regression
- [x] Commit Phase 1
- [x] Push Phase 1

## Phase 2: ADC DMA Input (`AudioInputDma`)
- [x] Add firmware-local `AudioInputDma` class + integration
- [x] Replace direct ISR ADC reads with DMA latest-sample path
- [x] Keep one-pot-per-control-tick behavior
- [x] Build `pico` and `pico2`
- [x] Run fixed test script
- [x] Confirm no pot-stability regression and no added distortion
- [x] Commit Phase 2
- [x] Push Phase 2

## Phase 3: DMA/Control Handoff Stabilization
- [x] Add deterministic post-resume settle/discard logic
- [x] Add input DC blocker (default enabled)
- [x] Keep averaging as optional A/B switch
- [x] Build `pico` and `pico2`
- [x] Run fixed test script
- [x] Confirm idle whine is same or improved (target: improved)
- [x] Commit Phase 3
- [x] Push Phase 3

## Phase 4: Optional DAC SPI DMA Output (Only if Needed)
- [x] Decide if Phase 4 is needed based on Phase 3 results
- [ ] Add firmware-local `AudioOutputDmaSpi` class + integration
- [ ] Keep A/B fallback path available during validation
- [ ] Build `pico` and `pico2`
- [ ] Run fixed test script
- [ ] Confirm clear audible benefit vs Phase 3
- [ ] Commit Phase 4
- [ ] Push Phase 4
Note: Currently treated as skipped unless new issues require it.

## Phase 5: Cleanup + RC
- [x] Remove temporary switches not needed for release
- [x] Keep minimal test/debug controls
- [x] Build `pico` and `pico2`
- [ ] Final full fixed test script
- [x] Create `v0.2-rc1` branch/tag point
- [ ] Commit Phase 5
- [ ] Push Phase 5

---

## Fixed Test Script (Run After Every Phase)
- [ ] `normal`, no input cable, 20s: rate whine (`none/low/med/high`) + drift note
- [ ] `drypass`, input A shorted to ground, 20s: rate whine
- [ ] `normal`, constant tone input: sweep pot1/pot2/pot3 (15s each), rate distortion
- [ ] Check serial counters for runaway/abnormal values
- [ ] Write a short pass/fail summary in commit message or notes
