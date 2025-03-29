package Drone;

public class Main {
    public static void main(String[] args) {
        System.out.println("Simulating Bereshit's Landing:");
        double vs = 24.8;
        double hs = 932.0;
        double dist = 181000.0;
        double ang = 58.3;
        double alt = 13748.0;
        double time = 0.0;
        double dt = 1.0;
        double acc = 0.0;
        double fuel = 121.0;
        double weight = 165.0 + fuel;
        System.out.println("vs , desired_vs , hs , desired_hs , alt , ang , acc , fuel");

        Bereshit_101.BereshitLander lander = new Bereshit_101.BereshitLander(
                vs, hs, ang, fuel, 0.7, dist, alt, time, dt, acc, weight
        );

        PIDController pidVS = new PIDController(0.014, 0.000000003, 0.2);
        PIDController pidAngle = new PIDController(0.314, 0.00003, 0.13);

        while (lander.getAlt() > 0) {
            double desiredVs = Bereshit_101.getDesiredVs(lander.getAlt());
            double desiredHs = Bereshit_101.getDesiredHs(lander.getAlt());
            double desiredAng = Bereshit_101.getDesiredAngle(lander.getAlt());

            if (lander.getTime() % 10 == 0 || lander.getAlt() < 100) {
                System.out.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                        lander.getVs(), desiredVs,
                        lander.getHs(), desiredHs,
                        lander.getAlt(), lander.getAng(),
                        lander.getAcc(), lander.getFuel());
            }

            double throttleDelta = pidVS.update((lander.getVs() - desiredVs) + (lander.getHs() - desiredHs), lander.getDt());
            double angleDelta = pidAngle.update(desiredAng - lander.getAng(), lander.getDt());

            lander.increaseThrust(throttleDelta);
            lander.increaseAngle(angleDelta);

            lander.computeNextStep();
        }

        System.out.printf("Landing ended: alt=%.2f, vs=%.2f, hs=%.2f, fuel=%.2f\n",
                lander.getAlt(), lander.getVs(), lander.getHs(), lander.getFuel());

        if (lander.getVs() >= -2.5 && lander.getHs() <= 2.5 && lander.getFuel() >= 50) {
            System.out.println("Landing successful! 🚀");
        } else {
            System.out.println("Crashed! 💥");
        }
    }
}
