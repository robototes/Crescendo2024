package swervelib.parser;

import edu.wpi.first.math.controller.PIDController;
import swervelib.parser.deserializer.PIDFRange;

/** Hold the PIDF and Integral Zone values for a PID. */
public class PIDFConfig {

	/** Proportional Gain for PID. */
	public double p;
	/** Integral Gain for PID. */
	public double i;
	/** Derivative Gain for PID. */
	public double d;

	public double kS;
	public double kV;
	public double kA;
	/** Integral zone of the PID. */
	public double iz;

	/** The PIDF output range. */
	public PIDFRange output = new PIDFRange();

	/** Used when parsing PIDF values from JSON. */
	public PIDFConfig() {}

	public PIDFConfig(double p, double i, double d, double kS, double kV, double kA, double iz) {
		this.p = p;
		this.i = i;
		this.d = d;
		this.kS = kS;
		this.kV = kV;
		this.kA = kA;
		this.iz = iz;
	}

	public PIDFConfig(double p, double i, double d, double kS, double kV, double kA) {
		this(p, i, d, kS, kV, kA, 0);
	}

	/**
	 * PIDF Config constructor to contain the values.
	 *
	 * @param p P gain.
	 * @param i I gain.
	 * @param d D gain.
	 * @param kV kV gain
	 * @param iz Intergral zone.
	 */
	public PIDFConfig(double p, double i, double d, double kV, double iz) {
		this(p, i, d, 0, kV, 0, iz);
	}

	/**
	 * PIDF Config constructor to contain the values.
	 *
	 * @param p P gain.
	 * @param i I gain.
	 * @param d D gain.
	 * @param f kV gain.
	 */
	public PIDFConfig(double p, double i, double d, double kV) {
		this(p, i, d, kV, 0);
	}

	/**
	 * PIDF Config constructor to contain the values.
	 *
	 * @param p P gain.
	 * @param i I gain.
	 * @param d D gain.
	 */
	public PIDFConfig(double p, double i, double d) {
		this(p, i, d, 0, 0);
	}

	/**
	 * PIDF Config constructor to contain the values.
	 *
	 * @param p P gain.
	 * @param d D gain.
	 */
	public PIDFConfig(double p, double d) {
		this(p, 0, d, 0, 0);
	}

	/**
	 * Create a PIDController from the PID values.
	 *
	 * @return PIDController.
	 */
	public PIDController createPIDController() {
		return new PIDController(p, i, d);
	}
}
