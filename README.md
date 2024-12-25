# Force-Modulated Compliant Controller for Human Gait Simulation

## Overview
This project implements a Force-Modulated Compliant Hip (FMCH) controller for human walking simulation using the SCONE simulation environment. FMCH is a bio-inspired model designed to achieve postural control in human walking through muscle length and ground reaction force modulation.

## Controller Implementations

### HamstringsController1
The first implementation uses muscle tendon unit (MTU) length for control:
```
Activation = c * GRF * (L - L0) / (mass * gravity)
```
where:
- L = fiber_length + tendon_length
- L0 = rest length parameter (optimizable)
- c = gain parameter (optimizable)
- GRF = normalized ground reaction force

Key characteristics:
- Uses absolute MTU length values
- Muscle activation can be negative when MTU length is shorter than rest length
- Basic implementation focusing only on hamstring muscles
- Optimization requires approximately 73 generations for stable walking

### HamstringsController2
An improved version using normalized fiber lengths:
```
Activation = c * GRF * max(L - L0, 0) / (mass * gravity)
```
where:
- L = normalized_fiber_length
- L0 = normalized rest length parameter
- c = gain parameter with expanded range

Improvements:
- Uses normalized muscle fiber length for better parameter optimization
- Activation is clamped to zero when length is below rest length
- Active only during early stance phase
- More robust optimization process requiring 150-250 generations
- Enhanced parameter ranges based on empirical testing

### Ham-RFController1
Introduces rectus femoris control alongside hamstrings:

Key features:
- Controls both hamstrings and rectus femoris muscles
- Uses same activation formula as HamstringsController2
- Independent gain parameters for each muscle group
- Optimization parameters customized for each muscle type
- Improved stability through coordinated muscle control

### Ham-RF-GMController1 & Ham-RF-GMController2
Latest implementations with comprehensive muscle control:

New features:
- Adds gluteus medius control
- Muscle-specific control functions
- Enhanced ground reaction force handling
- Optional length-dependent activation conditions
- Separate optimization parameters for each muscle group
- Improved stability through three-muscle coordination

## Parameter Optimization

Each controller version includes optimizable parameters:
- Gain coefficient (c): Controls the strength of the force-length response
- Rest length (L0): Defines the reference length for activation calculation
- Alpha: Scaling factor for final activation (fixed per muscle in most versions)

Parameter optimization has evolved through iterations:
- Initial versions: Wide ranges with simple constraints
- Later versions: Muscle-specific ranges based on empirical results
- Latest versions: Refined ranges with individual muscle considerations

## Future Work
- Further refinement of gluteus medius control parameters
- Implementation and integration of additional muscle reflex controllers
- Further optimization of initial parameters
<!-- - Development of adaptive parameter optimization
- Enhanced stability analysis and performance metrics
- Integration with full-body control systems -->

## Technical Notes
- All controllers use normalized ground reaction forces
- Activation calculations account for gravity and mass scaling
- Optional velocity-dependent terms are available but not currently used
- Data logging capabilities for all muscle activations and control signals

Note: This is an ongoing research project, and further improvements and validations are in progress. Parameter values and ranges may be adjusted based on optimization results and empirical testing.

<!-- ## Future Work
- Implementation of gluteus medius hip control
- Further optimization of initial parameters
- Integration testing of all controller components

Note: This is an ongoing research project, and further improvements and validations are in progress. -->
