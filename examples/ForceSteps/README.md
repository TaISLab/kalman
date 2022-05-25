# Kalman Filter Example 2: Leg tracking respect to a moving roller

This example illustrates the use of the Kalman filter library to track relative position of a leg respect to a roller.

## State
Leg state is given by a 2-vector containing the elements

* _x_: x-position (relative to roller)
* _y_: y-position (relative to roller)

## System Model
The system model defining the evolution of the system state over time is fairly straight forward and is defined in `SystemModelLeg.hpp` along with the control vector.
The system takes a velocity vector (dx,dy) as control input. Leg then moves along the vector.

## Measurement Models
Measurements of the system state are taken by a position-estimator which can be noisy or even lose readings.
Models and measurement vector is defined in `PositionMeasurementModelLeg.hpp`.

## Results
### Estimated Trajectory
![Estimate](leg_estimate.png)

### Euclidean distance error in each time-step
![Error](leg_error.png)
