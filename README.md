# Force-Modulated Compliant Controller for Human Gait Simulation

## Overview
This project implements a Force-Modulated Compliant Hip (FMCH) controller for human walking simulation using the SCONE simulation environment. FMCH is a bio-inspired model designed to achieve postural control in human walking.

## Controller Implementations

### HamstringsController1
The first implementation uses the following activation formula:
```
Activation = c * GRF * (L - L0) / (mass * gravity)
```

Key characteristics:
- Muscle activation can be negative when MTU length (L) is shorter than rest length (L0)
- Optimization requires approximately 73 generations to achieve stable walking for 20 seconds
- Basic implementation of the force-modulated control concept

### HamstringsController2
An improved version using normalized length values:
```
Activation = c * GRF * max(L - L0, 0) / (mass * gravity)
```

Improvements:
- Uses normalized muscle length for better parameter optimization
- Activation is clamped to zero when length is below rest length
- Active only during early stance phase
- Optimization requires 150-250 generations depending on initial parameters
- Wider range of normalized length values potentially facilitates parameter optimization

### Ham-RFController1
Latest implementation with enhanced functionality:
- Incorporates rectus femoris control during late stance phase
- Reduces required optimization generations
- Requires optimal initial parameters
- Planned integration with gluteus medius hip control

## Future Work
- Implementation of gluteus medius hip control
- Further optimization of initial parameters
- Integration testing of all controller components

Note: This is an ongoing research project, and further improvements and validations are in progress.
