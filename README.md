# TD3-ADRC: Intelligent Control for Quadrotors

Twin Delayed Deep Deterministic Policy Gradient + Active Disturbance Rejection Control for quadrotor UAVs.

## Overview

This project implements the **TD3-ADRC** intelligent controller based on "Intelligent Control for Quadrotors Based on a Novel Method: TD3-ADRC" (Cai et al., 2026). The method combines deep reinforcement learning (TD3) with enhanced ADRC using a parameterized tanh function for superior disturbance rejection.

**Key Innovation**: Achieves **37.28% improvement** over traditional ADRC through:
- Twin Q-networks (mitigate overestimation)
- Delayed policy updates (training stability)
- Novel parameterized tanh function (smooth, continuous nonlinearity)
- Online ADRC parameter adaptation via learned policy

## Features

- 12-state nonlinear quadrotor dynamics model
- Enhanced ADRC with parameterized tanh function
- TD3 deep reinforcement learning for parameter optimization
- MATLAB/Simulink simulation environment
- Blender 3D visualization pipeline
- MCP integration for interactive control

## Quick Start

```matlab
% Setup paths
setup_paths;

% Run comparison demo
demo_comparison;
```

## Project Structure

```
TD3-ADRC/
├── params/           % Configuration parameters
├── src/
│   ├── dynamics/     % Quadrotor dynamics model
│   ├── control/
│   │   ├── adrc/     % ADRC components
│   │   └── td3_adrc/ % TD3 + ADRC integration
│   ├── estimation/   % Disturbance observers
│   ├── visualization/% Plotting and export
│   └── mcp_tools/    % MCP server integration
├── simulink/         % Simulink models
├── demos/            % Demonstration scripts
├── tests/            % Unit tests
├── results/          % Output figures and animations
└── blender/          % 3D visualization assets
```

## Use Cases

### 1. Wind Gust Disturbance Rejection
Quadrotor maintaining hover under time-varying wind disturbances.

### 2. Aggressive Maneuver Tracking
Track sinusoidal + step attitude commands with rapid transitions.

### 3. PX4 Hardware-in-the-Loop
Validation on 3-DOF test bench with PX4 flight controller.

## Performance Results

| Controller | Roll IAE | Pitch IAE | Yaw IAE | Improvement |
|------------|----------|-----------|---------|-------------|
| PID        | 80.27    | 70.13     | 93.48   | baseline    |
| ADRC       | 138.08   | 81.75     | 98.42   | -           |
| DDPG-ADRC  | 85.34    | 59.27     | 67.68   | +22%        |
| **TD3-ADRC** | **33.50** | **38.43** | **51.68** | **+37%** |

## Requirements

- MATLAB R2023b or later
- Deep Learning Toolbox
- Reinforcement Learning Toolbox
- Simulink (optional)
- Blender 3.6+ (for visualization)

## References

Cai et al. (2026). "Intelligent Control for Quadrotors Based on a Novel Method: TD3-ADRC"

## License

MIT License - See LICENSE file for details.
