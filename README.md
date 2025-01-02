# Force-Modulated Compliant Controller for Human Gait Simulation

## Overview
This project implements a Force-Modulated Compliant Hip (FMCH) controller, a bio-inspired model designed for postural control in human walking simulation. The implementation leverages the SCONE simulation environment and modulates control based on muscle length and ground reaction forces. While traditional PD controllers (implemented as DofReflex in SCONE) have proven effective for postural control, this project explores the potential of FMCH controllers as an alternative approach.

## Controller Implementations

### HamstringsController1
Initial implementation focusing on MTU (Muscle Tendon Unit) length control:

\[
\text{Activation} = \frac{c \times \text{GRF} \times (L - L_0)}{m \times g}
\]

**Where:**
- \(L = \text{fiber\_length} + \text{tendon\_length}\)
- \(L_0\) = optimizable rest length parameter
- \(c\) = optimizable gain parameter
- \(\text{GRF}\) = normalized ground reaction force
- \(m\) = mass
- \(g\) = gravity

Key characteristics:
- Utilizes absolute MTU length measurements
- Permits negative muscle activation when MTU length falls below rest length
- Focuses exclusively on hamstring muscle control
- Operates in conjunction with H1922RS2v3_HamstringsController1.scone, which disables DofReflex for hamstrings and pelvis tilt during stance phase

### HamstringsController2
Enhanced implementation utilizing normalized fiber lengths:

Here are the two formulas rewritten in a more formal mathematical notation:

\[
\text{Activation} = \frac{c \times \text{GRF} \times \max(L - L_0, 0)}{m \times g}
\]

**Where:**
- \(L = \text{normalized\_fiber\_length}\)
- \(L_0\) = normalized rest length parameter
- \(c\) = gain parameter with expanded optimization range
- \(\text{GRF}\) = normalized ground reaction force
- \(m\) = mass
- \(g\) = gravity

<!-- ```
Activation = c * GRF * max(L - L0, 0) / (mass * gravity)
```
where:
- L = normalized_fiber_length
- L0 = normalized rest length parameter
- c = gain parameter with expanded optimization range -->

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

<!-- New update at 2025/1/2 -->

### Ham-RF-GMController2

The **Ham-RF-GMController2** is a combined controller designed to manage the activation of three key muscles during a gait cycle: the **Hamstrings**, **Rectus Femoris (RF)**, and **Gluteus Medius (GM)**. This controller integrates the functionalities of three individual controllers (`HamstringsController2`, `RFController2`, and `GMController1`) to optimize muscle activation during different phases of the stance phase. The primary goal of this controller is to improve gait efficiency by dynamically adjusting muscle activations based on ground reaction forces (GRF) and muscle lengths.

#### Key Features:
- **Hamstrings**: The controller removes the `DofReflex` but retains the `MuscleReflex` for the hamstrings.
- **Rectus Femoris**: The controller uses length threshold mechanisms, with a focus on early stance phase activation.
- **Gluteus Medius**: The controller removes the `DofReflex` and `LengthReflex` but can optionally retain the `VelocityReflex`.

The controller works by calculating muscle activations based on the FMC formula:

\[
\text{activation} = c \times \text{GRF} \times (\text{L} - \text{L0}, 0)
\]

where:
- \(c\) is a scaling factor,
- \(\text{GRF}\) is the ground reaction force,
- \(\text{L}\) is the current muscle length,
- \(\text{L0}\) is the rest length of the muscle.

#### Individual Controllers

##### 1. HamstringsController2

The **HamstringsController2** is responsible for controlling the hamstrings muscle. It removes the `DofReflex` but retains the `MuscleReflex`. The controller ensures that the hamstrings are activated based on the muscle length and GRF during the stance phase.

###### Key Characteristics:
- **Reflexes**: Removes `DofReflex`, retains `MuscleReflex`.
- **Activation Formula**: Uses the standard activation formula based on GRF and muscle length.
- **Performance**: Achieved 90 generations of optimization.

##### 2. Rectus Femoris Controllers (RFController1 and RFController2)

The **Rectus Femoris** is realized by two different controllers: **RFController1** and **RFController2**. Both controllers use FMC control strategy, but they differ in their activation timing and length threshold mechanisms.

###### RFController1:
- **Activation Timing**: Works during the entire stance phase.
- **Activation Formula**: Uses the formula \(\text{activation} = c \times \text{GRF} \times (\text{L} - \text{L0}, 0)\).
- **Performance**: Achieved 12-15 generations of optimization.

###### RFController2:
- **Activation Timing**: Works only during the early stance phase.
- **Length Threshold**: Sets the rest length (\(\text{L0}\)) as a threshold. If the muscle length reaches \(\text{L0}\), the activation output becomes 0, regardless of subsequent muscle length changes.
- **Performance**: Achieved 9 generations of optimization.

###### Comparison:
- **RFController1** activates the muscle throughout the stance phase.
- **RFController2** introduces a length threshold mechanism that stops activation once the muscle reaches its rest length and activates on early stance phase, which is more biological than **RFController1**.

##### 3. Gluteus Medius Controllers (GMController1 and GMController2)

The **Gluteus Medius** is realized by two different controllers: **GMController1** and **GMController2**. Both controllers remove the `DofReflex` and `MuscleReflex`, but **GMController2** introduces the same length threshold mechanism from **RFController2**.

###### GMController1:
- **Reflexes**: Removes `DofReflex` and `MuscleReflex`, but can retain `VelocityReflex` with certain parameters.
- **Activation Formula**: Uses the formula \(\text{activation} = c \times \text{GRF} \times (\text{L} - \text{L0}, 0)\).
- **Performance**: Achieved 540 generations with velocity reflex and 457 generations without velocity reflex.

###### GMController2:
- **Reflexes**: Removes `DofReflex` and `MuscleReflex`, but can retain `VelocityReflex` with certain parameters.
- **Activation Formula**: Similar to **RFController2**, but the mechanics inherited from **RFController2** do not work effectively in **GMController2** because the muscle length will also increase while initial rest length increasing.
- **Performance**: Achieved 457 generations with velocity reflex and 559 generations without velocity reflex.

###### Comparison:
- **GMController2** attempts to implement a similar length threshold mechanism as **RFController2**, but it is less effective due to the change of muscle length in gait cycle.

#### Conclusion

The **Ham-RF-GMController** is a sophisticated combination of three individual controllers, each optimized for specific muscles and gait phases. By integrating the functionalities of `HamstringsController2`, `RFController2`, and `GMController1`, this controller provides a comprehensive solution for managing muscle activations during walking. The key differences between **RFController1** and **RFController2** lie in their activation timing and length threshold mechanisms, while **GMController1** and **GMController2** differ in their handling of the `VelocityReflex`. Overall, this controller represents a significant step forward in gait optimization, achieving 685 generations of combined optimization.

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