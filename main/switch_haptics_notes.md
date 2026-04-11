# Switch Haptics Implementation Notes (RetroOnyx)

## Overview

This implementation decodes Nintendo Switch HD rumble packets and maps them to dual DRV2625 LRA drivers.

The Switch protocol provides **dual-band haptic data** (low + high frequency components), but this hardware cannot faithfully reproduce both simultaneously without introducing artifacts.

This implementation prioritizes **perceptual feel over raw signal fidelity**.

---

## Key Constraints

### 1. Hardware limitation (LRA + DRV2625)

- LRA is a **resonant system**, not a waveform synthesizer  
- DRV2625 in this configuration:
  - supports discrete frequency modes (e.g. ~160 Hz, ~320 Hz)
  - does **not support smooth multi-frequency blending**
- Rapid frequency switching results in:
  - mechanical settling
  - tactile discontinuities
  - perceivable “pulsing”

---

### 2. Switch packet vs actuator reality

Switch sends:

```
low frequency + amplitude
high frequency + amplitude
```

This represents **haptic intent**, not a literal playback instruction.

Attempting to render both bands directly (via timeslicing) results in:

- LO → HI → LO → HI pattern  
- perceived as **distinct pulses**  
- does not match Switch Pro Controller feel  

---

## Why timeslicing was rejected

Tested approach:

```
4 ms @ 160 Hz
4 ms @ 320 Hz
(repeat)
```

Observed result:

- “4 distinct pulses” per rumble event  
- even at 2 ms → still perceptible as faster pulsing  
- actuator exposes frequency switching  

**Conclusion:**

> Discrete band alternation does not produce smooth haptics on this hardware.

---

## Final Rendering Strategy

### Per decoded sample:

#### 1. Select frequency (dominant band)

```c
selected_freq = (hi_amp > lo_amp) ? hi_freq : lo_freq;
```

- Avoids frequency switching artifacts  
- Preserves strongest perceptual component  

---

#### 2. Compute amplitude (averaged)

```c
selected_amp = (hi_amp + lo_amp) * 0.5f;
```

- Prevents overdriving / saturation  
- Retains contribution from both bands  

---

#### 3. Apply output gain

```c
#define HAPTICS_OUTPUT_GAIN 3.0f
```

- Tuned for:
  - good dynamic range  
  - no excessive strength  
  - stable feel  

---

#### 4. Playback routing

- Left only → drive left  
- Right only → drive right  
- Both active → **broadcast mode**  

Broadcast is used because:

- it keeps both DRV2625 devices synchronized  
- avoids reconfiguration instability  
- already proven reliable in this system  

---

## Key Insight

> The Switch provides more haptic detail than this hardware can physically render.

So instead of reproducing the signal literally:

👉 We **compress the signal into a single clean actuator drive**

---

## Result

This approach provides:

- smooth, continuous vibration  
- no pulsing artifacts  
- stable dual-motor behavior  
- perceptually similar feel to Switch Pro Controller  

---

## What we intentionally do NOT do

- ❌ LO/HI timeslicing  
- ❌ rapid frequency switching  
- ❌ dual-frequency synthesis  
- ❌ waveform reconstruction  

These were tested and produced worse results.

---

## Final Summary

| Feature | Implementation |
|--------|--------|
| Frequency | Dominant band |
| Amplitude | Average of bands |
| Gain | 3.0 |
| Dual motors | Broadcast |
| Timeslicing | ❌ Not used |

---

## Conclusion

This is a **hardware-constrained rendering model** that:

- sacrifices theoretical accuracy  
- in favor of correct tactile behavior  

### 👉 Result: *feels right beats decodes perfectly*
