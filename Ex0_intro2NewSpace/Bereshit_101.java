package Drone;
/**
 * This class represents the basic flight controller of the Bereshit space craft.
 * It includes constants for physical parameters and engine characteristics.
 * @author ben-moshe
 */
public class Bereshit_101 {
	public static final double WEIGHT_EMP = 165;  // Empty spacecraft weight (kg)
	public static final double WEIGHT_FULE = 420; // Fuel weight (kg)
	public static final double WEIGHT_FULL = WEIGHT_EMP + WEIGHT_FULE; // Total initial weight (kg)
// https://davidson.weizmann.ac.il/online/askexpert/%D7%90%D7%99%D7%9A-%D7%9E%D7%98%D7%99%D7%A1%D7%99%D7%9D-%D7%97%D7%9C%D7%9C%D7%99%D7%AA-%D7%9C%D7%99%D7%A8%D7%97
	public static final double MAIN_ENG_F = 430; // Main engine thrust (N)
	public static final double SECOND_ENG_F = 25; // Side engine thrust (N)
	public static final double MAIN_BURN = 0.15;  // Main engine fuel consumption (liters/sec)
	public static final double SECOND_BURN = 0.009; // Side engine fuel consumption (liters/sec)
	public static final double ALL_BURN = MAIN_BURN + 8*SECOND_BURN; // Total burn rate at full thrust

	/**
	 * Calculates maximum acceleration with all engines firing.
	 * @param weight Current weight of the spacecraft
	 * @return Maximum possible acceleration
	 */
	public static double accMax(double weight) {
		return acc(weight, true,8);
	}

	/**
	 * Calculates acceleration based on engine usage.
	 * @param weight Current weight
	 * @param main Whether main engine is active
	 * @param seconds Number of side engines active (0–8)
	 * @return Acceleration value
	 */
	public static double acc(double weight, boolean main, int seconds) {
		double t = 0;
		if(main) {t += MAIN_ENG_F;}
		t += seconds*SECOND_ENG_F;
		double ans = t/weight;
		return ans;
	}
	/**
	 * Inner class representing the Bereshit lander state and simulation behavior.
	 * * Includes position, velocity, angle, thrust, fuel, and time.
	 */
	public static class BereshitLander {
		private double vs;     // Vertical speed (m/s)
		private double hs;     // Horizontal speed (m/s)
		private double ang;    // Orientation angle (degrees)
		private double fuel;   // Remaining fuel (liters)
		private double NN;     // Throttle [0,1]
		private double dist;   // Distance to landing site (m)
		private double alt;    // Altitude above surface (m)
		private double time;   // Time since start (s)
		private double dt;     // Time step (s)
		private double acc;    // Current acceleration (m/s^2)
		private double weight; // Current spacecraft weight (kg)

		/**
		 * Constructor to initialize the lander's full state.
		 */
		public BereshitLander(double vs, double hs, double ang, double fuel, double NN,
							  double dist, double alt, double time, double dt,
							  double acc, double weight) {
			this.vs = vs;
			this.hs = hs;
			this.ang = ang;
			this.fuel = fuel;
			this.NN = NN;
			this.dist = dist;
			this.alt = alt;
			this.time = time;
			this.dt = dt;
			this.acc = acc;
			this.weight = weight;
		}
		/**
		 * Increases or decreases thrust, clamped to [0, 1].
		 */
		public void increaseThrust(double inc) {
			this.NN = Math.max(0, Math.min(1, this.NN + inc));
		}
		/**
		 * Adjusts the angle within [0, 90] degrees.
		 */
		public void increaseAngle(double angInc) {
			this.ang = Math.max(0, Math.min(90, this.ang + angInc));
		}

		/**
		 * Advances the simulation one step forward using physical equations.
		 */
		public void computeNextStep() {
			double ang_rad = Math.toRadians(ang);// Convert angle to radians
			double h_acc = Math.sin(ang_rad) * acc;// Horizontal acceleration from thrust
			double v_acc = Math.cos(ang_rad) * acc;// Vertical acceleration from thrust
			double vacc = Moon.getAcc(hs);// Gravitational pull from Moon

			time += dt;
			double dw = dt * ALL_BURN * NN;// Fuel used this step

			if (fuel > 0) {
				fuel -= dw;
				weight = WEIGHT_EMP + fuel;
				acc = NN * accMax(weight);
			} else {
				acc = 0;
			}

			v_acc -= vacc; // Apply lunar gravity
			if (hs > 0) hs -= h_acc * dt;// Reduce horizontal speed

			dist -= hs * dt;// Update horizontal position
			vs -= v_acc * dt;// Update vertical speed
			alt -= dt * vs;// Update altitude

			if (hs < 2.5) hs = 0; // Consider horizontal speed zero if very small
		}

		// ---------------------- Getters & Setters ----------------------
		public double getVs() { return vs; }
		public void setVs(double vs) { this.vs = vs; }

		public double getHs() { return hs; }
		public void setHs(double hs) { this.hs = hs; }

		public double getAng() { return ang; }
		public void setAng(double ang) { this.ang = ang; }

		public double getFuel() { return fuel; }
		public void setFuel(double fuel) { this.fuel = fuel; }

		public double getNN() { return NN; }
		public void setNN(double NN) { this.NN = NN; }

		public double getDist() { return dist; }
		public double getAlt() { return alt; }
		public double getTime() { return time; }
		public double getDt() { return dt; }
		public double getAcc() { return acc; }
		public double getWeight() { return weight; }
	}
	public static double getDesiredHs(double alt) {
		double minAlt = 2000.0;
		double maxAlt = 30000.0;
		if (alt < minAlt) {
			return 0.0;
		} else if (alt > maxAlt) {
			return 1700.0;
		} else {
			double norm = (alt - minAlt) / (maxAlt - minAlt);
			return norm * 1700.0;
		}
	}

	public static double getDesiredVs(double alt) {
		if (alt > 8000.0) return 30.0;
		else if (alt > 500.0) return 24.0;
		else if (alt > 300.0) return 12.0;
		else if (alt > 100.0) return 6.0;
		else if (alt > 50.0) return 3.0;
		else return (alt > 25.0 ? 2.0 : 1.0);
	}

	public static double getDesiredAngle(double alt) {
		if (alt > 1500.0) return 60.0;
		else if (alt > 1200.0) return 50.0;
		else if (alt > 1000.0) return 30.0;
		else return 0.0;
	}

}

