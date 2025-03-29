package Drone;

/**
 * Simulates Bereshit's landing and prints detailed telemetry in your required format:
 * <alt>, <vs>, <hs>, <hAcc>, <vAcc>, ang: <ang> ,dang: <dang>, <acc>, <power>, <fuel>
 */
public class Main {
    public static void main(String[] args) {
        System.out.println("Simulating Bereshit's Landing:");

        // Initial state
        double vs = 24.8;               // vertical speed
        double hs = 932.0;              // horizontal speed
        double dist = 181000.0;         // distance to landing point
        double ang = 58.3;              // angle
        double alt = 13748.0;           // altitude
        double time = 0.0;              // time
        double dt = 1.0;                // time step
        double acc = 0.0;               // acceleration
        double fuel = 121.0;            // fuel
        double weight = 165.0 + fuel;   // weight

        // Print header for reference
        System.out.println("vs , desired_vs , hs , desired_hs , alt , ang , acc , fuel");

        // Create lander
        Bereshit_101.BereshitLander lander = new Bereshit_101.BereshitLander(
                vs, hs, ang, fuel, 0.7, dist, alt, time, dt, acc, weight
        );

        // PID controllers
        PIDController pidVS = new PIDController(0.014, 0.000000003, 0.2);
        PIDController pidAngle = new PIDController(0.314, 0.00003, 0.13);

        // Simulation loop
        while (lander.getAlt() > 0) {
            // Get desired states from controller
            double desiredVs = Bereshit_101.getTargetVerticalSpeed(lander.getAlt());
            double desiredHs = Bereshit_101.getTargetHorizontalSpeed(lander.getAlt());
            double desiredAng = Bereshit_101.getTargetAngle(lander.getAlt());

            // Calculate control changes
            double dang = desiredAng - lander.getAng();
            double throttleDelta = pidVS.update(
                    (lander.getVs() - desiredVs) + (lander.getHs() - desiredHs),
                    lander.getDt()
            );
            double angleDelta = pidAngle.update(dang, lander.getDt());

            // Apply control updates
            lander.increaseThrust(throttleDelta);
            lander.increaseAngle(angleDelta);

            // Print in requested format:
            System.out.printf("%.15f, %.15f, %.1f, 0.0, 0.0, ang: %.1f ,dang: %.1f, %.15f, %.15f, %.15f\n",
                    lander.getAlt(),
                    lander.getVs(),
                    lander.getHs(),
                    lander.getAng(),
                    dang,
                    lander.getAcc(),
                    lander.getNN(),
                    lander.getFuel()
            );

            lander.computeNextStep();
        }

        // Final print
        System.out.printf("Landing ended: alt=%.2f, vs=%.2f, hs=%.2f, fuel=%.2f\n",
                lander.getAlt(), lander.getVs(), lander.getHs(), lander.getFuel());
    }
}
