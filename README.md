# Initial research on MPC and non lineal optimisation for real-time application.
![MPC simulation](testAnimated.gif)

## Part of the autonomous vehicle project

### Weights exploration

Minimise lateral error starting away from the reference trajectory
1. Reducing time step helps to improve the trackability (minimise lateral error) when starting from an offset position relative to the reference. It comes with computational cost.
2. Increasing lateral error weight seems to be the most effective way to reduce lateral error.
3. Decreasing speed helps.

