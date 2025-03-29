# Beresheet Lander Simulation

This Java project simulates the autonomous landing of the **Beresheet** spacecraft on the Moon.  
It models the lander's physical dynamics and uses **PID controllers** to control its descent.

## 🚀 Project Features

- Simulates **vertical and horizontal speeds**, angle, acceleration, fuel usage, and thrust power.
- Uses **PID control loops** for vertical speed and angle corrections.
- Outputs real-time telemetry in detailed CSV-like format.
- Modular design: logic is separated into the `Main`, `Bereshit`, `Moon`and `PIDController` classes.


