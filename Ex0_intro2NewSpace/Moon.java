package Drone;

/**
 * This class represents physical properties of the Moon relevant to the Bereshit landing simulation.
 * Includes gravitational acceleration and an approximation for varying gravity based on horizontal velocity.
 */
public class Moon {

	/**
	 * The radius of the Moon in meters.
	 * Source: https://he.wikipedia.org/wiki/%D7%94%D7%99%D7%A8%D7%97
	 */
	public static final double RADIUS = 3475 * 1000; // meters

	/**
	 * The average gravitational acceleration on the surface of the Moon (m/s^2).
	 */
	public static final double ACC = 1.622; // m/s^2

	/**
	 * Approximate orbital speed at the Moon's equator (m/s).
	 * Used to approximate gravity reduction due to horizontal velocity.
	 */
	public static final double EQ_SPEED = 1700; // m/s

	/**
	 * Computes the effective gravitational acceleration as a function of horizontal speed.
	 * The faster the horizontal speed, the smaller the gravitational effect,
	 * simulating a simplistic model of orbital mechanics.
	 *
	 * @param speed The horizontal speed of the spacecraft (m/s)
	 * @return The adjusted gravitational acceleration (m/s^2)
	 */
	public static double getAcc(double speed) {
		double n = Math.abs(speed) / EQ_SPEED; // Normalize speed
		double ans = (1 - n) * ACC; // Reduce gravity proportionally
		return ans;
	}
}
