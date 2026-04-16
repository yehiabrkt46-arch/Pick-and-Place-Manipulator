
# Robotics & Computer Vision — Pick and Place Manipulator

Modelling, trajectory planning, and closed-loop control of a 3-DOF RRR 
serial manipulator performing a pick and place task. Implemented in 
MATLAB using the Robotics Toolbox.

## Problem Overview

A 3-DOF revolute manipulator is tasked with moving its gripper from a 
starting configuration to a target ball position (x=0.6, y=0.6, z=0.7m) 
in world coordinates. Two control strategies are implemented and compared.

## What's Covered

**Kinematics**
- Robot modelled using Denavit-Hartenberg parameters via `SerialLink`
- Forward kinematics to compute gripper pose from joint angles
- Inverse kinematics with position-only mask `[1 1 1 0 0 0]` — the 
  3-DOF manipulator is underactuated relative to full 6-DOF task space

**Joint-Space Control (Q3)**
- Quintic polynomial trajectory via `jtraj` — zero velocity and 
  acceleration at start and end
- Proportional closed-loop controller tracking joint-space trajectory
- Jacobian computed at each step for end-effector velocity analysis
- Smooth bell-shaped velocity profile confirmed by differential kinematics

**Cartesian-Space Control (Q4)**
- Straight-line end-effector trajectory via `ctraj`
- Proportional controller using pseudoinverse Jacobian for joint velocities
- Singularity instability observed — near-infinite joint velocities when 
  Jacobian loses rank, causing erratic motion

## Key Finding

Joint-space control is more computationally stable and simpler to 
implement. Cartesian-space control offers more predictable straight-line 
paths but breaks down near singularities without damping or 
singularity-robust pseudoinverse methods.

## Dependencies

- MATLAB
- Robotics Toolbox for MATLAB (Corke, 2017)

## Files

| File | Description |
|------|-------------|
| `C23042477.m` | Full MATLAB implementation — all 4 questions |
