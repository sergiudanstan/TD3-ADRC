# TD3-ADRC Project Guide

## Project Overview

This project implements TD3-ADRC (Twin Delayed Deep Deterministic Policy Gradient + Active Disturbance Rejection Control) for quadrotor UAV attitude control. The method combines deep reinforcement learning with enhanced ADRC using a novel parameterized tanh function.

## Key Concepts

### ADRC Components
1. **Tracking Differentiator (TD)** - Smooths reference signals
2. **Extended State Observer (ESO)** - Estimates total disturbance as extended state
3. **Nonlinear State Error Feedback (NLSEF)** - Generates control law

### TD3 Algorithm
- Twin Q-networks to reduce overestimation bias
- Delayed policy updates for stability
- Target policy smoothing for variance reduction

### Novel tanh Function
Replaces discontinuous `fal` function with smooth parameterized tanh:
```matlab
y = -(delta^(alpha*e) - delta^(-alpha*e)) / (delta^(alpha*e) + delta^(-alpha*e));
```

## File Structure

- `params/` - Parameter files (quadrotor, ADRC, TD3)
- `src/dynamics/` - Quadrotor dynamics model
- `src/control/adrc/` - ADRC components (TD, ESO, NLSEF, tanh)
- `src/control/td3_adrc/` - TD3 agent and networks
- `demos/` - Demonstration scripts
- `blender/` - 3D visualization

## Running Simulations

```matlab
setup_paths;              % Initialize paths
demo_comparison;          % Run controller comparison
demo_wind_disturbance;    % Wind gust rejection demo
demo_trajectory_tracking; % Aggressive maneuver demo
```

## Key Parameters

### Quadrotor
- Mass: 1.4 kg
- Arm length: 0.225 m
- Inertia: diag([0.0211, 0.0219, 0.0366]) kg·m²

### TD3
- Actor learning rate: 1e-4
- Critic learning rate: 1e-3
- Discount factor: 0.99
- Policy delay: 2

### ADRC
- Observer bandwidth: 50-100 rad/s
- Controller bandwidth: 10-20 rad/s

## MCP Integration

Use `td3_adrc_mcp_tools.m` for interactive control via MCP server.

## Blender Export

```matlab
export_to_blender(flight_data, 'results/flight_data.json');
```

Then in Blender:
```bash
blender --background blender/drone_scene.blend --python blender/scripts/animate_flight.py
```
