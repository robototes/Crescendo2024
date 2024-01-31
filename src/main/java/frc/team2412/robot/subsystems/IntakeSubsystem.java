package frc.team2412.robot.subsystems;

import static frc.team2412.robot.Hardware.INTAKE_DISTANCE_SENSOR;
import static frc.team2412.robot.Hardware.INTAKE_MOTOR_1;
import static frc.team2412.robot.Hardware.INTAKE_MOTOR_2;
import static frc.team2412.robot.subsystems.IntakeSubsystem.IntakeConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.subsystems.IntakeSubsystem.IntakeConstants.GamePieceType;
import java.awt.Color;
import java.util.Map;

public class IntakeSubsystem extends SubsystemBase {
	// CONSTANTS
	public static class IntakeConstants {
		// speeds
		public static final double INTAKE_HOLD_SPEED = 0.12;
		public static final double INTAKE_IN_SPEED = 1.0;
		public static final double INTAKE_OUT_SPEED = -0.1;
		public static final double INTAKE_FAST_OUT_SPEED = -0.7;

		public static final int INTAKE_COLOR_THRESHOLD = 10;
		public static final double LED_PURPLE = 0.91;
		public static final double LED_YELLOW = 0.69;

		// enums
		public static enum GamePieceType {
			CUBE(
					new Color(64, 108, 81),
					LED_PURPLE,
					9), // first color is from the color sensor (desaturated green), second is more true to
			// color
			CONE(
					new Color(84, 127, 42),
					LED_YELLOW,
					7.05), // first color is from the color sensor (desaturated green), second is more true to
			// color
			NONE(new Color(0, 0, 0), 0, 8.55); // black

			public final Color color;
			public final double ledColor;
			public final double distanceFromSensor;

			GamePieceType(Color color, double ledColor, double distanceFromSensor) {
				this.color = color;
				this.ledColor = ledColor;
				this.distanceFromSensor = distanceFromSensor;
			}
		}
	}

	// Network Tables
	NetworkTableInstance NTInstance;
	NetworkTable NTDevices;

	StringPublisher gamePieceTypePublisher;
	DoublePublisher distancePublisher;
	StringPublisher colorPublisher;
	DoublePublisher currentSpeedPublisher;

	// HARDWARE
	private final CANSparkMax motor1;
	private final CANSparkMax motor2;
	// private final ColorSensorV3 colorSensor;
	private final DigitalInput distanceSensor;

	// Shuffle Board

	private static GenericEntry intakeNotMovingEntry =
			Shuffleboard.getTab("Intake").addPersistent("Not Moving", true).withSize(4, 1).getEntry();

	private static GenericEntry intakeSpeedEntry =
			Shuffleboard.getTab("Intake")
					.addPersistent("Intake In Speed", INTAKE_IN_SPEED)
					.withSize(4, 1)
					.withWidget(BuiltInWidgets.kNumberSlider)
					.withProperties(Map.of("Min", 0, "Max", 1))
					.getEntry();

	private static GenericEntry distanceSensorEntry =
			Shuffleboard.getTab("Intake")
					.addPersistent("Distance Sensor Value", 0)
					.withSize(1, 1)
					.getEntry();
	// CONSTRUCTOR
	public IntakeSubsystem() {
		motor1 = new CANSparkMax(INTAKE_MOTOR_1, MotorType.kBrushless);
		motor2 = new CANSparkMax(INTAKE_MOTOR_2, MotorType.kBrushless);

		resetMotors();

		// colorSensor = new ColorSensorV3(Port.kOnboard);
		distanceSensor = new AnalogInput(INTAKE_DISTANCE_SENSOR);

		// Network Tables
		NTInstance = NetworkTableInstance.getDefault();

		NTDevices = NTInstance.getTable("Devices");

		gamePieceTypePublisher = NTDevices.getStringTopic("Game Piece").publish();
		colorPublisher = NTDevices.getStringTopic("color").publish();
		distancePublisher = NTDevices.getDoubleTopic("Distance").publish();
		currentSpeedPublisher = NTDevices.getDoubleTopic("Current Speed").publish();

		gamePieceTypePublisher.set("NONE");
		colorPublisher.set("no color");
		distancePublisher.set(0.0);
		currentSpeedPublisher.set(0.0);
	}

	// METHODS

	public void resetMotors() {

		motor1.restoreFactoryDefaults();
		motor2.restoreFactoryDefaults();

		motor1.setIdleMode(IdleMode.kBrake);
		motor2.setIdleMode(IdleMode.kBrake);

		// need cause motors run opposite direciton.
		motor1.setInverted(true);
		motor2.setInverted(true);

		motor1.setSmartCurrentLimit(20);
		motor2.setSmartCurrentLimit(20);

		motor1.burnFlash();
		motor2.burnFlash();
	}

	/**
	 * Sets the speed of the intake motors to a specified value.
	 *
	 * @param speed Percent output to set the motors to run with.
	 */
	public void setSpeed(double speed) {
		motor1.set(speed);
		motor2.set(-speed);
	}

	/** Gets the current speed of intake. */
	public double getSpeed() {
		return motor1.getEncoder().getVelocity();
	}

	/**
	 * Decides a speed to use based on whether not there is an object inside intake to hold. Used for
	 * IntakeDefaultCommand.
	 *
	 * @return Motor holding speed if there is an object to keep secure, otherwise 0.
	 */
	public double getHoldSpeed() {
		return (!hasObject() ? 0 : INTAKE_HOLD_SPEED);
	}

	/** Runs the motors inwards */
	public void intakeIn() {
		setSpeed(intakeSpeedEntry.getDouble(INTAKE_IN_SPEED));
	}

	/** Runs the motors outwards */
	public void intakeOut() {
		setSpeed(INTAKE_OUT_SPEED);
	}

	/** Runs the motors outwards */
	public void intakeFastOut() {
		setSpeed(INTAKE_FAST_OUT_SPEED);
	}

	/** Stops the motors */
	public void intakeStop() {
		setSpeed(0);
	}

	/**
	 * Compares the current color sensor value to color values corresponding to game pieces in order
	 * to detect what is being intaked.
	 *
	 * @return The game piece detected.
	 */
	public GamePieceType detectType() {
		// 	if (colorSensorEquals(CUBE.color)) {
		// 		return GamePieceType.CUBE;
		// 	} else if (colorSensorEquals(CONE.color)) {
		// 		return GamePieceType.CONE;
		// 	}
		return GamePieceType.NONE;
	}

	/**
	 * Compares the current color sensor value to the color specified as a parameter.
	 *
	 * @param color Color to compare the color sensor value to.
	 * @return Whether or not the color sensors value matches the color target.
	 */
	public boolean colorSensorEquals(Color color) {
		// 	// r
		// 	if (colorSensor.getRed() <= (color.getRed() + INTAKE_COLOR_THRESHOLD)
		// 			&& colorSensor.getRed() >= (color.getRed() - INTAKE_COLOR_THRESHOLD)) {
		// 		// g
		// 		if (colorSensor.getGreen() <= (color.getGreen() + INTAKE_COLOR_THRESHOLD)
		// 				&& colorSensor.getGreen() >= (color.getGreen() - INTAKE_COLOR_THRESHOLD)) {
		// 			// b
		// 			if (colorSensor.getBlue() <= (color.getBlue() + INTAKE_COLOR_THRESHOLD)
		// 					&& colorSensor.getBlue() >= (color.getBlue() - INTAKE_COLOR_THRESHOLD)) {
		// 				return true;
		// 			}
		// 		}
		// 	}
		return false;
	}
	/**
	 * Checks whether or not the game piece is secured.
	 *
	 * @return True if the game piece is secured, false if not.
	 */
	public boolean isSecured() {
		// return (getDistance() < GamePieceType.CONE.distanceFromSensor
		// 		|| (detectType() == GamePieceType.CUBE
		// 				&& getDistance() < GamePieceType.CUBE.distanceFromSensor));

		return (getDistance() <= GamePieceType.NONE.distanceFromSensor);
	}

	/**
	 * Gets the distance of whatever's in front of the distance sensor
	 *
	 * @return Distance object is from sensor.
	 */
	public double getDistance() {
		// equation found from docs to convert voltage to cm
		return Math.pow(distanceSensor.getAverageVoltage(), -1.2045)
				* 27.726; // gets approximately the correct values for both game pieces
	}
	/**
	 * Checks whether or not the game piece is inside intake. Used for knowing when to stop outtaking.
	 *
	 * @return True if piece is inside intake, false if not.
	 */
	public boolean hasObject() {
		// 24 is distance for when object is inside intake
		return getDistance() < 24;
	}

	// TODO: good? test? need for driveteam.
	/**
	 * Checks whether or not intake has secured game piece based off speed, if within a small speed
	 * threshold, returns true. Otherwise false.
	 *
	 * @return Whether or not gamepiece is about secured
	 */
	public boolean isSpeedNearStopped() {
		return (getSpeed() < 0.025 && getSpeed() > -0.025);
	}

	@Override
	public void periodic() {
		// gamePieceTypePublisher.set(detectType().toString());
		// colorPublisher.set(colorSensor.getColor().toString());
		// distancePublisher.set(getDistance());
		currentSpeedPublisher.set(getSpeed());

		intakeNotMovingEntry.setBoolean(isSpeedNearStopped());
		// distanceSensorEntry.setDouble(getDistance());
	}
}
