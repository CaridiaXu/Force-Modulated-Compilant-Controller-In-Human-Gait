# Force-Modulated Compliant Controller for Human Gait Simulation

## Overview
This project implements a Force-Modulated Compliant Hip (FMCH) controller, a bio-inspired model designed for postural control in human walking simulation. The implementation leverages the SCONE simulation environment and modulates control based on muscle length and ground reaction forces. While traditional PD controllers (implemented as DofReflex in SCONE) have proven effective for postural control, this project explores the potential of FMCH controllers as an alternative approach.

## Controller Implementations

### HamstringsController1
Initial implementation focusing on MTU (Muscle Tendon Unit) length control:
```
Activation = c * GRF * (L - L0) / (mass * gravity)
```
where:
- L = fiber_length + tendon_length
- L0 = optimizable rest length parameter
- c = optimizable gain parameter
- GRF = normalized ground reaction force

Key characteristics:
- Utilizes absolute MTU length measurements
- Permits negative muscle activation when MTU length falls below rest length
- Focuses exclusively on hamstring muscle control
- Operates in conjunction with H1922RS2v3_HamstringsController1.scone, which disables DofReflex for hamstrings and pelvis tilt during stance phase

### HamstringsController2
Enhanced implementation utilizing normalized fiber lengths:
```
Activation = c * GRF * max(L - L0, 0) / (mass * gravity)
```
where:
- L = normalized_fiber_length
- L0 = normalized rest length parameter
- c = gain parameter with expanded optimization range

Enhancements:
- Implements normalized muscle fiber length for improved parameter optimization
- Enforces non-negative activation through zero-clamping
- Restricts activation to early stance phase
- Features empirically optimized parameter ranges
- Integrates with H1922RS2v3_HamstringsController2.scone, removing both DofReflex and MuscleReflex for hamstrings during stance phase

### Ham-RFController1
Extends control to include rectus femoris alongside hamstrings:

Key features:
- Manages both hamstring and rectus femoris muscle groups
- Inherits optimized parameters from HamstringsController2 for hamstring control
- Applies consistent activation formula across both muscle groups
- Implements muscle-specific parameter optimization
- Functions with H1922RS2v3_Ham-RFController1.scone, which eliminates stance phase DofReflex for both muscle groups

### Ham-RFController2
Refined implementation with enhanced rectus femoris control:

Key features:
- Adopts HamstringsController1's activation formula for rectus femoris, enabling negative activation below rest length
- Incorporates optimized rectus femoris control parameters
- Replicates stance phase behavior of rectus femoris through single FMC controller
- Works in tandem with H1922RS2v3_Ham-RFController2.scone, removing both DofReflex and MuscleReflex during stance phase

### Ham-RF-GMController1
Experimental version incorporating gluteus medius control:

Key features:
- Introduces gluteus medius control mechanism
- Implements modified activation formula for gluteus medius:
  ```Activation = -c * GRF * max(L - L0, 0) / (mass * gravity)```
- Generates negative output similar to DofReflex behavior for 3 muscles
- Pairs with H1922RS2v3_Ham-RF-GMController1.scone, disabling stance phase DofReflex for all three muscles

### Ham-RF-GMController2
Advanced experimental implementation:

Key features:
- Achieves non-DofReflex-equivalent output during stance phase through optimized control
- Operates alongside H1922RS2v3_Ham-RF-GMController2.scone, eliminating both reflex types for all muscles during stance

## Parameter Optimization

Each controller implementation features:
- Gain coefficient (c): Modulates force-length response magnitude
- Rest length (L0): Establishes activation reference threshold
- Alpha: Fixed per-muscle activation scaling factor

The optimization process has evolved through:
- Initial phase: Basic constraints with broad parameter ranges
- Intermediate phase: Muscle-specific empirical optimization
- Current phase: Refined individual muscle parameterization

## Future Work
- Optimization of gluteus medius control parameters
- Development of additional muscle reflex controllers
- Further refinement of initialization parameters

<!-- ## Technical Notes
- Implements normalized ground reaction force calculations
- Incorporates gravity and mass scaling in activation computations
- Supports optional velocity-dependent control terms
- Provides comprehensive data logging for muscle activation and control signals -->

Note: This project remains under active development, with ongoing improvements and validation studies. Control parameters continue to be refined based on empirical testing and optimization results.