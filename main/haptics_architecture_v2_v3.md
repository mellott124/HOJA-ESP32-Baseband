# Haptics Architecture Overview (V2 vs V3)

## Summary

The Nintendo Switch rumble system encodes two simultaneous haptic components per actuator: a low-frequency component and a high-frequency component, each with independent amplitude. This implies a layered waveform model rather than a single-frequency resonant drive.

The current controller (V2) uses a closed-loop haptic driver, which limits output to a single effective frequency at a time. As a result, the implementation simplifies Switch rumble into a dominant frequency approximation.

A future controller (V3) would require direct waveform synthesis and a bipolar drive stage to more faithfully reproduce Switch-style layered haptics.

---

## Switch Rumble Model

Switch rumble packets provide:

- Low-frequency component (frequency + amplitude)
- High-frequency component (frequency + amplitude)
- Continuous real-time updates (no waveform IDs or canned effects)

Conceptually, this corresponds to a signal of the form:

x(t) = A_lo * sin(2π f_lo t) + A_hi * sin(2π f_hi t)

This represents **simultaneous multi-frequency content**, not time-sliced playback.

---

## V2 Haptics Architecture (Current)

### Hardware

- DRV2625 closed-loop haptic driver  
- Single LRA / haptic reactor  

### Characteristics

- Driver locks to a single effective resonant frequency  
- Automatically tracks and optimizes resonance  
- Cannot reproduce multiple independent frequency components simultaneously  

### Implementation Strategy

1. Decode Switch rumble packet:
   - Extract f_lo, A_lo, f_hi, A_hi  

2. Reduce to a single output:
   - Select dominant frequency (typically highest energy component)  
   - Combine amplitudes (average or weighted)  

3. Drive actuator:
   - Send single frequency + amplitude to DRV2625  

### Result

- Stable and efficient output  
- Good approximation of overall intensity  
- Loss of layered texture (no simultaneous hi/lo content)  

---

## V3 Haptics Architecture (Proposed)

### Hardware

- MCU-generated waveform (ESP32 or equivalent)  
- Bipolar output stage (H-bridge or bridged amplifier)  
- Direct drive of LRA / reactor  

### Architecture

Switch rumble packet  
        ↓  
Decode packet fields (f_lo, A_lo, f_hi, A_hi)  
        ↓  
Haptics synthesis (MCU):  
- Low-frequency oscillator  
- High-frequency oscillator  
- Amplitude scaling  
- Envelope shaping  
- Output limiting  
        ↓  
PWM / DAC output  
        ↓  
H-bridge / bridged amplifier  
        ↓  
Haptic actuator  

### Synthesis Model

- Two oscillators (low + high frequency)  
- Continuous phase accumulation (DDS-style)  
- Summed waveform output  
- Real-time parameter updates per packet  

### Benefits

- True layered haptic output  
- Simultaneous low-frequency motion + high-frequency texture  
- Closer match to Switch “HD Rumble” behavior  

### Tradeoffs

- Increased firmware complexity  
- Requires careful current and voltage limiting  
- Loss of automatic resonance tracking  
- Greater responsibility for actuator safety and tuning  

---

## V2 vs V3 Comparison

| Feature | V2 (DRV2625) | V3 (H-bridge drive) |
|--------|-------------|--------------------|
| Drive model | Closed-loop resonance | Open-loop waveform |
| Frequency output | Single effective frequency | Dual-frequency (simultaneous) |
| Waveform control | Driver-managed | MCU-controlled |
| Texture fidelity | Approximate | High |
| Complexity | Low | Moderate |
| Safety handling | Built-in | Firmware-managed |

---

## Conclusion

The current V2 implementation is fundamentally limited by the use of a closed-loop haptic driver, which enforces single-frequency behavior. While this produces a stable and usable result, it cannot reproduce the layered nature of Switch rumble.

A V3 design using direct waveform synthesis and a bipolar drive stage would enable significantly more accurate haptic reproduction, at the cost of increased system complexity and responsibility for drive control.
