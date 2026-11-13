# Converting the robot's 15 A battery limit to the sim's Pololu current limit (by hand)

The robot's motors get 15 A of the 20 A battery fuse (the Pi, up to 3 A peak, the flyback, 1 A, and the rest take the other 5 A). The sim uses the geared Pololu motor, the robot the direct-drive BLDC, so a Pololu amp is not a BLDC amp. The conversion goes through the **same motion**: the same force at each wheel and the same wheel speed.

Assumptions: steady state (no wheel acceleration), motor inductance ignored, the driver passes power from the battery to the motor without loss.

## Constants

- Pololu, at the wheel: Kt_p = Kb_p = 0.255 N·m/A, R_p = 2.18 Ω
- BLDC: Kt_b = Kb_b = 0.0265 N·m/A, R_b = 0.24 Ω
- Battery: V_bus = 16.8 V; the motors' share of the fuse I_batt,max = 15 A

## 1. Same motion: same force F at each wheel and same wheel speed ω

```
Torque at the wheel:   F r = Kt_p i_p = Kt_b i_b
=>  i_b = n i_p,       n = Kt_p / Kt_b = 0.255 / 0.0265 = 9.62
```

The BLDC carries 9.6 times the motor current for the same force (the Pololu's 30:1 gearbox).

## 2. BLDC power per wheel (the voltage it needs times its current)

```
V_b = R_b i_b + Kb_b ω
P_b = V_b i_b = R_b n² i_p²  +  Kb_b n ω i_p
              = (22.2 Ω) i_p² + Kt_p ω i_p        (Kb_b n = Kt_p: the mechanical power F·v is the same)
              =  copper loss  +  mechanical power
```

R_b n² = 0.24 × 9.62² = 22.2 Ω, against the Pololu's own 2.18 Ω: the BLDC loses 10 times as much in its windings for the same force.

## 3. Battery current (the driver passes power)

```
I_batt = Σ P_b / V_bus      over the 4 wheels
```

The fuse sees battery current, not motor current: a motor that is barely turning needs little voltage, so a large motor current can come from a small battery current.

## 4. Even split of the Pololu total S = Σ|i_p| (i_p = S/4 on every wheel, all driving)

```
I_batt = [ 4 · 22.2 · (S/4)² + Kt_p ω S ] / V_bus
       = [ 5.55 S² + Kt_p ω S ] / 16.8
```

## 5. Set I_batt = 15 A and solve the quadratic for S

```
5.55 S² + Kt_p ω S − 15 · 16.8 = 0          (15 · 16.8 = 252 W)

S = [ −Kt_p ω + √( (Kt_p ω)² + 4 · 5.55 · 252 ) ] / (2 · 5.55)
```

## Results

| Case | Kt_p·ω | Pololu limit S |
|---|---|---|
| Standstill (ω = 0) | 0 | √(252 / 5.55) = **6.7 A** |
| Driving at ω = 20 rad/s | +5.1 V | (−5.1 + 75.0) / 11.1 = **6.3 A** |
| Braking at ω = 20 rad/s (power returns) | −5.1 V | (5.1 + 75.0) / 11.1 = **7.2 A** |

**Worked check at standstill:**
- Pololu: 6.7 A in total, so 1.7 A per wheel, which makes 14 N per wheel.
- BLDC, for the same 14 N: 16 A per wheel, with copper loss 0.24 × 16² ≈ 62 W per wheel, so 250 W in total.
- Battery current: 250 W / 16.8 V ≈ **15 A**.

## How this relates to the 7.4 A used in the sim

- **By hand:** 6.3–7.2 A, depending on whether the wheels are driving or braking; 6.7 A at standstill.
- **The 7.4 A in the sim** is the same physics averaged over real motion: 87,060 samples, 1 ms apart, from three controllers on five training paths, with the BLDC battery current fitted as 0.229 S² + 0.343 S. It sits a little above the standstill number for three reasons:
  - driving and braking are mixed
  - the four currents are rarely split evenly
  - the BLDC's lighter direct-drive wheel needs less current to spin up, which the hand formula leaves out
- **An uneven split lowers the limit:** all the current on one wheel gives 22.2 S² instead of 5.55 S², so S = √(252 / 22.2) = 3.4 A. That is the absolute worst case.

**Summary:** by hand, about **6.7 A** (±0.5 A with speed); the sim uses **7.4 A**, the average over how the robot actually moves. The 6.7 A is the cleaner number to quote, and slightly more conservative.
