# MPC for Autonomous Vehicle Control in Simplified Environment


## Introduction

Model Predictive Control is an optimization problem to solve the needs of following a trajectory. The solution to this optimization problem suggests the optimal path in which an autonomous vehicle can follow. The MPC involves simulating different actuator inputs and predicting the trajectory with the minimum cost.

## Summarized Control Idealogy

- At each timestep, we must update the actuator inputs to always have the most optimized trajectory. 
- Once the lowest cost trajectory is found, execute the actuation commands.
- At each timestep, we update to find a new optimal trajectory. In this way, we calculate inputs over a future horizon.
- Updating the model will provide the most accurate estimation of the required actuations over time.

## Cost Function

**Minimize**:

- Offset from the center of the lane (reference, or desired, state)
- Deviation from reference velocity
- Distance from the target location
