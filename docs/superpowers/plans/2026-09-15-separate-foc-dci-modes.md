# Separate FOC DCI Modes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Split mode 24 calibration, mode 28 integrated calibration/current-loop, and mode 29 run-only current-loop into independent state machines.

**Architecture:** Keep generic Clarke/Park/SVPWM/fault/PWM helpers shared, but remove the common DCI variant state machine. Add `foc_dcal24` and `foc_drun29`; reduce `foc_dci` to mode 28 only. Mode 29 receives one immutable calibration snapshot from mode 24 and samples TMRA_1 for the startup rotor angle.

**Tech Stack:** C99 embedded firmware for HC32F460, Keil MDK project, 20 kHz FOC ISR.

---

## File Structure

- `ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/foc_dcal24.h`
  Public mode 24 API, state/event codes, calibration result struct, observables.
- `ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/foc_dcal24.c`
  Private mode 24 zero-offset/BETA/ALPHA state machine and handoff snapshot.
- `ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/foc_drun29.h`
  Public mode 29 API, state/event codes, PI configuration, observables.
- `ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/foc_drun29.c`
  Private mode 29 startup rotor-angle sampling and current-loop state machine.
- `ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/foc_dci.h`
  Mode 28-only declarations; remove variant and mode 24/29 APIs.
- `ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/foc_dci.c`
  Mode 28-only state machine; remove variant branches and CAL24 event.
- `ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/foc.h`
  Include the two new headers.
- `ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/foc.c`
  Dispatch three independent running flags.
- `ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/foc_obs.c`
  Independent mode 24/28/29 event handling and periodic printing.
- `ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/dev_comm_runner.c`
  Independent Start/Stop calls for modes 24/28/29.
- `ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/template/source/main.c`
  VOFA mappings distinguish mode 28 from mode 29.
- `ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/template/MDK/template - 副本.uvprojx`
  Add both new `.c` files to both targets.

---

### Task 1: Create Independent Mode 24 Module

**Files:**

- Create: `ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/foc_dcal24.h`
- Create: `ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/foc_dcal24.c`

- [ ] **Step 1: Define the private handoff contract**

The result is copied by value to mode 29. Mode 29 has no access to mode 24 internals.

```c
typedef struct {
    uint8_t valid;
    int32_t offset;
    float   zero_u_ma;
    float   zero_v_ma;
    float   zero_w_ma;
} foc_dcal24_result_t;

uint8_t Foc_Dcal24_GetResult(foc_dcal24_result_t *result);
```

- [ ] **Step 2: Implement mode 24 states and events**

Use these public observables and event codes:

```c
volatile uint8_t g_dcal24_running;
volatile uint8_t g_dcal24_state;
volatile uint8_t g_dcal24_evt;
volatile uint16_t g_dcal24_beta_hw;
volatile uint16_t g_dcal24_alpha_hw;
volatile int32_t g_dcal24_moved;
volatile int32_t g_dcal24_offset;
volatile float g_dcal24_zero_u_ma;
volatile float g_dcal24_zero_v_ma;
volatile float g_dcal24_zero_w_ma;
volatile float g_dcal24_volt_v;
```

State order: `IDLE -> ZERO -> BETA -> ALPHA -> DONE` and `FAULT_OC`. `ALPHA` computes `offset = mod(hw * enc_dir, CPR)`, stores the immutable result, stops PWM, and raises `DCAL24_EVT_DONE`. `foc_obs.c` returns the runner to mode 0. Do not call `Foc_Calib_*` from mode 24; its zero window is private.

- [ ] **Step 3: Verify mode 24 symbols**

Run: `rg -n "Foc_Dcal24_(Start|Step|Stop|GetResult)|DCAL24_EVT_DONE" ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/foc_dcal24.*`

Expected: all four functions and `DCAL24_EVT_DONE` are present; no `Foc_Dci_*` symbols are referenced.

### Task 2: Create Independent Mode 29 Module

**Files:**

- Create: `ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/foc_drun29.h`
- Create: `ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/foc_drun29.c`

- [ ] **Step 1: Define mode 29-only API and observables**

Use the `g_drun29_*` prefix for every public mode 29 value. Keep the same PID values, current reference, ramp, delta parameters, and statistical windows as the former mode 29 path.

```c
void Foc_Drun29_InitPids(void);
void Foc_Drun29_Start(void);
void Foc_Drun29_Stop(void);
void Foc_Drun29_Step(const stc_i_data_t *pData);
```

- [ ] **Step 2: Copy mode 24 result and sample current rotor angle**

At startup, reject an invalid result before changing PWM state. Then read the live encoder:

```c
hw_now = TMRA_GetCountValue(CM_TMRA_1);
s_rotor_count = Foc_Core_ModPos((int32_t)hw_now * dir - result.offset,
                                (int32_t)ENCODER_CPR);
s_encoder_prev_hw = hw_now;
s_encoder_initialized = 1u;
```

Subsequent ISR deltas are wrap-safe, clamped to +/-32 counts, multiplied by `enc_dir`, and folded into `s_rotor_count`. This ensures a rotor moved during mode 0 starts from the live angle at mode 29 entry.

- [ ] **Step 3: Implement the private current loop**

The run loop follows the former DCI RUN path: delta ramp, corrected current copy, Park feedback, PI outputs, inverse Park, SVPWM, mean/peak/error/speed statistics. It uses its own static PI states and no `Foc_Calib_*` state.

- [ ] **Step 4: Verify startup semantics**

Run: `rg -n "Foc_Dcal24_GetResult|TMRA_GetCountValue|g_drun29_running|Foc_Dci_" ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/foc_drun29.c`

Expected: `Foc_Dcal24_GetResult` and `TMRA_GetCountValue` occur in startup; no `Foc_Dci_` references occur.

### Task 3: Reduce `foc_dci` to Mode 28 Only

**Files:**

- Modify: `ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/foc_dci.h`
- Modify: `ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/foc_dci.c`

- [ ] **Step 1: Remove variant API**

Delete `g_dci_variant`, `g_dcal_offset_valid`, `DCI_VARIANT_*`, `DCI_EVT_CAL24_DONE`, `Foc_Dcal24_Start()`, and `Foc_Drun29_Start()`.

- [ ] **Step 2: Make mode 28 transitions unconditional**

`Foc_Dci_Start()` directly resets and enters `DCI_STEP_CALIB`. At ALPHA completion, mode 28 always stores its offset and enters `DCI_STEP_RUN`; it never returns to mode 0 as calibration-only mode.

- [ ] **Step 3: Verify no split-mode remnants**

Run: `rg -n "DCI_VARIANT|g_dci_variant|DCI_EVT_CAL24_DONE|Foc_Dcal24|Foc_Drun29|g_dcal_offset_valid" ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/foc_dci.*`

Expected: no matches.

### Task 4: Wire Independent Dispatch, Runner, Observability, and VOFA

**Files:**

- Modify: `ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/foc.h`
- Modify: `ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/foc.c`
- Modify: `ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/dev_comm_runner.c`
- Modify: `ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/foc_obs.c`
- Modify: `ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/template/source/main.c`

- [ ] **Step 1: Dispatch independent modes**

`Foc_Init()` initializes both mode 28 and mode 29 PI instances. The ALIGN ISR path checks, in order, mode 20, 25, 26, 27, mode 24, mode 28, mode 29, then plain mode 23. Mode 29 may reuse `FOC_MODE_ALIGN` as an ISR routing mode because its running flag is private and checked before fallback.

- [ ] **Step 2: Make runner transitions explicit**

Every stop path calls all three functions:

```c
if (g_dcal24_running) Foc_Dcal24_Stop();
if (g_dci_running)    Foc_Dci_Stop();
if (g_drun29_running) Foc_Drun29_Stop();
```

Mode cases call exactly one corresponding Start function.

- [ ] **Step 3: Separate observation events**

Replace the shared `DCI_EVT_CAL24_DONE` handling with `DCAL24_EVT_DONE`. Add separate mode 29 event and periodic-data blocks using `g_drun29_*`. Mode 28 blocks remain driven only by `g_dci_running`.

- [ ] **Step 4: Separate VOFA sources**

Replace `g_dci_running ? ...` choices in the mode 28/29 channel family with explicit checks:

```c
if (g_drun29_running)       { /* use g_drun29_* */ }
else if (g_dci_running)     { /* use g_dci_* */ }
else                        { /* retain existing voltage/current observation */ }
```

Do not change the 14-channel frame order or scaling.

- [ ] **Step 5: Search for cross-mode leakage**

Run: `rg -n "g_dci_(running|state|id_ma|iq_ma|id_ref_ma|iq_ref_ma).*g_drun29|g_drun29_.*g_dci_|DCI_VARIANT|g_dcal_offset_valid" ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws`

Expected: no matches.

### Task 5: Update Keil Build

**Files:**

- Modify: `ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/template/MDK/template - 副本.uvprojx`

- [ ] **Step 1: Add both source files to both targets**

Immediately after the existing `foc_dci.c` file node in each target, add:

```xml
<File>
  <FileName>foc_dcal24.c</FileName>
  <FileType>1</FileType>
  <FilePath>..\..\ws\foc_dcal24.c</FilePath>
</File>
<File>
  <FileName>foc_drun29.c</FileName>
  <FileType>1</FileType>
  <FilePath>..\..\ws\foc_drun29.c</FilePath>
</File>
```

- [ ] **Step 2: Verify project insertion**

Run: `Select-String -LiteralPath 'ws_L_v.inmop_curloop_re\HC32F460_DDL_Rev3.3.0\projects\ev_hc32f460_lqfp100_v2\template\MDK\template - 副本.uvprojx' -Pattern 'foc_dcal24\.c|foc_drun29\.c'`

Expected: four matches, two per target.

### Task 6: Build and Final Verification

**Files:**

- No source edits unless a build error requires one.

- [ ] **Step 1: Run a clean source-symbol audit**

Run from repository root:

```powershell
rg -n "DCI_VARIANT|g_dci_variant|DCI_EVT_CAL24_DONE|g_dcal_offset_valid|Foc_Dcal24_StartWith|Foc_Drun29_StartWith" ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2
```

Expected exit code 1 and no matches.

- [ ] **Step 2: Build Keil targets**

Locate Keil at `C:\Keil_v5\UV4\UV4.exe`. Build the textual project:

```powershell
& 'C:\Keil_v5\UV4\UV4.exe' -b 'D:\WS_L_re\ws_L_v.inmop_curloop_re\HC32F460_DDL_Rev3.3.0\projects\ev_hc32f460_lqfp100_v2\template\MDK\template - 副本.uvprojx' -j0 -o 'D:\WS_L_re\build_separate_dci.log'
```

Expected: Keil exits with code 0 or 1 and the log contains `0 Error(s)`. If UV4 is unavailable, record that limitation and run the symbol/structural checks.

- [ ] **Step 3: Review the final diff**

Run: `git diff --check; git diff --stat`

Expected: no whitespace errors; the diff contains only the planned firmware and Keil files.

- [ ] **Step 4: Commit the implementation**

```powershell
git add -- ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/foc_dcal24.h ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/foc_dcal24.c ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/foc_drun29.h ws_L_v.inmop_curloop_re/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/foc_drun29.c
git commit -m "feat: separate FOC modes 24, 28, and 29"
```

Do not stage the unrelated modified `template - 副本 - 副本.uvoptx` file.

