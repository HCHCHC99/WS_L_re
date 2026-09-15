# Separate FOC Mode 24, 28, and 29

## Goal

Mode 24, mode 28, and mode 29 currently share one DCI state machine in
`foc_dci.c`. The behavior must be separated so each mode owns its own state
machine, runtime flags, statistics, events, and stop path.

The original workflow remains:

1. Start mode 24.
2. Mode 24 calibrates phase-current offsets and the encoder offset.
3. Mode 24 stops PWM and automatically returns to mode 0.
4. Mode 0 continues updating the observed rotor position.
5. Start mode 29.
6. Mode 29 uses the mode 24 calibration result and samples the current rotor
   angle at startup.
7. Mode 29 runs its own current loop from that sampled rotor angle.

## Non-Goals

- Do not change mode 20, 25, 26, 27, 30, 31, or 32 behavior.
- Do not change the Modbus, VOFA frame layout, PWM frequency, or sampling
  frequency.
- Do not make mode 29 perform its own BETA/ALPHA calibration.
- Do not remove low-level shared facilities that are not mode logic.

## Architecture

### Mode 24

Mode 24 moves into a new `ws/foc_dcal24.c/h` module.

It owns:

- the calibration state machine: zero-offset window, BETA, ALPHA, done, and
  over-current;
- its own running flag, state, event code, statistics, PI-free calibration
  state, and stop function;
- the public calibration handoff snapshot consumed by mode 29;
- automatic PWM stop after ALPHA locks the encoder offset;
- an event that `foc_obs.c` handles in main-loop context to switch back to
  mode 0.

The handoff snapshot contains the valid flag, encoder offset, and the three
phase-current offsets. The snapshot is mode 24 data; mode 29 copies it at
startup and does not reference mode 24 internal state afterward.

### Mode 28

Mode 28 remains in `ws/foc_dci.c/h`. The module becomes a self-contained
calibrate-plus-run implementation. All variant branches and `DCI_VARIANT_*`
definitions are removed.

Mode 28 retains its own current-loop state machine and public `g_dci_*`
observations. It no longer sets mode 24 completion events or checks a shared
mode variant.

### Mode 29

Mode 29 moves into a new `ws/foc_drun29.c/h` module.

It owns:

- the run-only current-loop state machine;
- its own running flag, PI instances, event codes, encoder frame, current
  references, statistics, observables, and stop function;
- a private copy of the mode 24 phase-current offsets and encoder offset;
- the 20 kHz current-loop ISR step.

At startup, mode 29 requires a valid mode 24 snapshot. It then reads TMRA_1
directly and computes:

```c
rot_start = mod((hw_now * enc_dir) - mode24_encoder_offset, CPR)
```

This is the actual rotor angle at mode 29 startup, not the angle captured when
mode 24 finished. Mode 29 initializes its private encoder accumulator from
this value and integrates subsequent hardware deltas in its own ISR.

### Allowed Shared Facilities

Only generic low-level facilities remain shared:

- Clarke/Park/SVPWM math;
- `foc_core` fault protection, PWM start/stop, modular arithmetic, and dq
  helpers;
- encoder hardware access;
- PID arithmetic utility.

No mode 24/28/29 state machine state, variant tag, running flag, event flag,
PI instance, statistics buffer, or completion event may be shared.

## Integration

`foc.h` includes the two new module headers. `foc.c` dispatches three
independent running flags:

- `g_dcal24_running` for mode 24;
- `g_dci_running` for mode 28 only;
- `g_drun29_running` for mode 29.

`dev_comm_runner.c` stops each old module explicitly and starts the module
selected by mode 24, 28, or 29. `Foc_Dcal24_Start()`, `Foc_Dci_Start()`, and
`Foc_Drun29_Start()` have no hidden coupling through a variant argument.

`foc_obs.c` handles each module through its own events and observable names.
Mode 24 completion prints and switches to mode 0. Mode 28 and mode 29 print
their own events and periodic statistics without reading each other's state.

The VOFA channels in `main.c` use mode 29's `g_drun29_*` data when
`g_drun29_running` is true and mode 28's `g_dci_*` data when
`g_dci_running` is true.

## Failure Behavior

- If mode 29 starts without a valid mode 24 snapshot, it prints an error,
  leaves PWM stopped, and does not run.
- Every mode keeps independent over-current handling. Over-current stops only
  that mode's output and records its own fault state/event.
- If a different mode is requested, `CommRunner_SetMode()` stops all currently
  active FOC mode modules before starting the requested module.
- The mode 24 snapshot remains sticky after returning to mode 0 so the rotor
  may be moved before starting mode 29.

## Verification

- Build the Keil target after adding both new source files to the project.
- Search for removed symbols such as `DCI_VARIANT_*` and confirm there are no
  references.
- Confirm mode 24 completion still emits the automatic-return-to-mode-0 event.
- Confirm mode 29 startup rejects when no mode 24 calibration exists.
- Inspect the mode 29 startup path to verify it reads the current TMRA_1 count
  rather than retaining the mode 24 lock-time relative frame.
