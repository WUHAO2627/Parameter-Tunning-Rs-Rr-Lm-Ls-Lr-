# Parameter Identification for General Purpose Inverter

> Beside this README, refer to the Presentation for more

## Table of Contents
[toc]

## 1. Project Overview
This repository implements the complete parameter identification algorithm for three-phase induction motors used in general purpose VFDs. Accurate motor parameters are critical for high-performance vector control (both FVC and SVC) and V/F control.

## 2. VFD Control Algorithms
The general purpose VFD supports four main control algorithms:
- SVC: Speed Sensorless Vector Control
- FVC: Speed Sensor-based Vector Control
- V/F: Volts per Hertz Control
- Parameter Identification Algorithm (this project)

### 2.1 Parameter Identification Workflow
The identification process is performed in three sequential steps:
1.  Stator Resistance (Rs) Identification: Motor at standstill
2.  Rotor Resistance (Rr) and Leakage Inductance (Lσ) Identification: Motor at standstill
3.  Mutual Inductance (Lm) and No-Load Current (I0) Identification: Motor running at no-load

## 3. Detailed Identification Algorithms
### 3.1 Stator Resistance (R_s) Identification
**Principle**: DC volt-ampere method. Apply DC excitation between two stator phases, measure voltage and current to calculate R_s.

**Formula**:
R_s =(U_DC * D)/I  * 1/2

Where:
U_DC: DC bus voltage
D: PWM duty cycle
I: Measured phase current

**Implementation Steps**:
1.  Configure PWM: 4kHz switching frequency, `TBPRD = 7500` (for ≤690V AC systems). Target current: 80% of VFD rated current.
2.  Enable PWM output, delay 2s (for stable continuous identification).
3.  Increase comparator value (`Comper`) until U-phase current reaches 4× target value.
4.  Delay 1024ms, accumulate 512 samples of DC bus voltage and 1024 samples of U-phase current.
5.  Further increase `Comper` until U-phase current reaches 4× target value again.
6.  Delay 1024ms, accumulate 512 samples of DC bus voltage, 512 samples of U-phase current, and 1024 samples of V-phase current.
7.  Calculate stator resistance using the two duty cycles:
    D_1 = (ComperL - RS_PRD/2)/(RS_PRD/2), D_2 = (Comper - RS_PRD/2)/{RS_PRD/2)
8.  Disable PWM, reset variables, and return to initial state.

### 3.2 Rotor Resistance (Rr) and Leakage Inductance (Lσ) Identification
**Principle**: When a step voltage is applied, since L_m >> Lσ, the magnetizing current can be neglected. The current rise rate is determined by the total leakage inductance.

**Formula**:
u_s = i_s(R_s + R_r) + 2Lσ di_s/dt

Solve the following two equations simultaneously:
2/3 *U_dc = I_u(U_s=U_dc) * (R_s + R_r) + 2Lσ *dI_u(U_s=U_dc)/dt)
0 = I_u(U_s=0)} * (R_s + R_r) + 2Lσ *dI_u(U_s=0)/dt

**Implementation Steps**:
1.  Configure PWM: 5MHz clock, 416.7Hz switching frequency, `TBPRD = 12000`, initial `CMPA = 200`.
2.  Delay 40ms, measure DC bus voltage, initialize variables.
3.  Send 7 voltage pulses per identification cycle, collect 14 U-phase current samples.
4.  Perform 6 consecutive identifications (40ms interval), remove max/min values, and average the results.
5.  Adjust `PwmCompareValue` if current is too high/low, and repeat the process.

### 3.3 Mutual Inductance (Lm) and No-Load Current (I0) Identification
**Principle**: At no-load, slip frequency approaches 0, rotor impedance $R_r/s$ approaches infinity (rotor winding open circuit).

**Formula**:
L_m = U_m * I_m * sin(theta) /I_m^2  * 1/2*pi*f - Lσ

| Symbol | Description | Unit |
|:------:|:------------|:----:|
| U_m | Fundamental voltage amplitude | V |
| I_m | Fundamental current amplitude | A |
| theta | Phase difference between voltage and current | rad |
| f | Fundamental frequency | Hz |

**Implementation Steps**:
1.  Initialize variables, target frequency: 80% of motor rated frequency.
2.  Enable PWM output, delay 1.6s.
3.  Wait until actual output frequency reaches 80% of rated frequency.
4.  Perform 120 consecutive identifications (240ms interval), average the results.
5.  Abort identification if motor speed drops below the set threshold.

## 4. Experimental Verification
### 4.1 Test Setup
| Equipment | Parameters |
|:---------:|:-----------|
| VFD | General purpose inverter |
| Motor | 7.5kW three-phase induction motor |
| Rated Parameters | 3AC 220V, 50Hz, 26A |

### 4.2 Test Results
| Test No. | Stator Resistance (Ω) | Rotor Resistance (Ω) | Leakage Inductance (H) | Mutual Inductance (H) | No-Load Current (A) |
|:--------:|:---------------------:|:--------------------:|:----------------------:|:---------------------:|:-------------------:|
| 1 | 0.216 | 0.120 | 0.00084 | 0.0482 | 8.16 |
| 2 | 0.217 | 0.118 | 0.00084 | 0.0483 | 8.15 |
| 3 | 0.213 | 0.114 | 0.00086 | 0.0483 | 8.15 |
| 4 | 0.213 | 0.113 | 0.00086 | 0.0484 | 8.13 |
| 5 | 0.214 | 0.113 | 0.00086 | 0.0485 | 8.12 |
| **Average** | **0.215** | **0.116** | **0.00085** | **0.0483** | **8.14** |

### 4.3 Conclusion
- The identification algorithm demonstrates high stability and repeatability.
- The VFD's AD sampling system provides low noise and high accuracy.

## 5. Repository Structure


## 6. Getting Started
1.  Configure the VFD hardware parameters (rated voltage, current, frequency).
2.  Run the automatic parameter identification sequence from the VFD keypad.
3.  Verify the identified parameters against the motor nameplate values.
4.  Use the identified parameters for vector control tuning.

## 7. Notes
- Ensure the motor is completely disconnected from the load before starting identification.
- The motor must be at standstill for Rs and Rr/Lσ identification steps.
- For high-power motors, ensure proper cooling during the no-load identification step.