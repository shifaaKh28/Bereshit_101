package Drone;

/**
 * PIDController class for controlling dynamic systems using PID logic.
 */
public class PIDController {
    private double kP; // Proportional gain
    private double kI; // Integral gain
    private double kD; // Derivative gain

    private double previousError; // Last error value (for derivative calculation)
    private double integralSum;   // Accumulated error over time (for integral term)
    private boolean isFirstRun;   // Flag to handle first iteration edge case

    /**
     * Constructor to initialize the PID controller with specific gain values.
     *
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.integralSum = 0;
        this.previousError = 0;
        this.isFirstRun = true;
    }

    /**
     * Updates the PID output based on current error and time step.
     *
     * @param currentError The current error value (target - actual)
     * @param deltaTime    Time interval (in seconds)
     * @return PID control output (not clamped)
     */
    public double update(double currentError, double deltaTime) {
        if (isFirstRun) {
            previousError = currentError;
            isFirstRun = false;
        }

        // Calculate derivative and accumulate integral
        double derivative = (currentError - previousError) / deltaTime;
        integralSum += currentError * deltaTime;

        // Compute the control output
        double output = kP * currentError + kI * integralSum + kD * derivative;

        // Store current error for next derivative calculation
        previousError = currentError;

        return output;
    }

    /**
     * Resets the internal state of the PID controller.
     * Useful when restarting or re-initializing control.
     */
    public void reset() {
        this.integralSum = 0;
        this.previousError = 0;
        this.isFirstRun = true;
    }
}
