package frc.team2412.robot.subsystems;

import static frc.team2412.robot.Hardware.*;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.Robot;
import java.util.Map;

public class IntakeSubsystem extends SubsystemBase {
	// Constants
	public static final double INTAKE_IN_SPEED = 0.6;
	public static final double INTAKE_REVERSE_SPEED = -0.7;
	public static final double INTAKE_REJECT_SPEED = -0.4;

	public static final double INDEX_UPPER_IN_SPEED = 1.0;
	public static final double INDEX_UPPER_REVERSE_SPEED = -1.0;

	public static final double INGEST_LOWER_IN_SPEED = 0.8;
	public static final double INGEST_LOWER_REVERSE_SPEED = -1.0;

	public static final double FEEDER_SHOOT_SPEED = 1.0;

	public static final double FEEDER_IN_SPEED = 1.0;
	public static final double FEEDER_REVERSE_SPEED = -1.0;

	private static final boolean enableFrontAndSideIntakes = true;

	@SuppressWarnings("ComplexBooleanConstant")
	private static final boolean FRONT_MOTOR_ENABLED = enableFrontAndSideIntakes && false;

	@SuppressWarnings("ComplexBooleanConstant")
	private static final boolean LEFT_MOTOR_ENABLED = enableFrontAndSideIntakes && true;

	@SuppressWarnings("ComplexBooleanConstant")
	private static final boolean RIGHT_MOTOR_ENABLED = enableFrontAndSideIntakes && true;

	// Motors
	private final CANSparkMax intakeMotorFront;
	private final CANSparkMax intakeMotorLeft;
	private final CANSparkMax intakeMotorRight;

	private final CANSparkFlex ingestMotor;
	private final CANSparkFlex indexMotorUpper;

	private final CANSparkFlex feederMotor;

	// Sensors
	private final SparkLimitSwitch indexSensor;
	private final SparkLimitSwitch feederSensor;
	private final DigitalInput feederSensorIR;
	// private final SparkLimitSwitch intakeFrontSensor;

	// debounce ! !
	@SuppressWarnings("UnusedVariable")
	private final Debouncer intakeFrontSensorDebouncer;

	@SuppressWarnings("UnusedVariable")
	private final Debouncer intakeRightSensorDebouncer;

	@SuppressWarnings("UnusedVariable")
	private final Debouncer intakeLeftSensorDebouncer;

	private final Debouncer feederSensorDebouncer;
	private final Debouncer feederSensorIRDebouncer;

	// private final SparkLimitSwitch intakeBackSensor;
	private final SparkLimitSwitch intakeLeftSensor;
	private final SparkLimitSwitch intakeRightSensor;

	// Shuffleboard

	private final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Intake");
	// speed
	private GenericEntry setIntakeInSpeedEntry;

	private GenericEntry setIndexInSpeedEntry;

	private GenericEntry setFeederInSpeedEntry;

	// sensor override
	private GenericEntry sensorOverride;

	// reject override
	private GenericEntry rejectOverride;

	public IntakeSubsystem() {

		intakeMotorFront =
				FRONT_MOTOR_ENABLED ? new CANSparkMax(INTAKE_MOTOR_FRONT, MotorType.kBrushless) : null;
		intakeMotorLeft =
				LEFT_MOTOR_ENABLED ? new CANSparkMax(INTAKE_MOTOR_LEFT, MotorType.kBrushless) : null;
		intakeMotorRight =
				RIGHT_MOTOR_ENABLED ? new CANSparkMax(INTAKE_MOTOR_RIGHT, MotorType.kBrushless) : null;

		ingestMotor = new CANSparkFlex(INGEST_MOTOR, MotorType.kBrushless);
		indexMotorUpper = new CANSparkFlex(INDEX_MOTOR_UPPER, MotorType.kBrushless);

		feederMotor = new CANSparkFlex(FEEDER_MOTOR, MotorType.kBrushless);

		indexSensor = indexMotorUpper.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
		feederSensor = feederMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
		feederSensorIR = new DigitalInput(FEEDER_SENSOR);

		// intakeFrontSensor =
		// intakeMotorFront.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
		// intakeBackSensor =
		// intakeMotorBack.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
		intakeLeftSensor =
				LEFT_MOTOR_ENABLED
						? intakeMotorLeft.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen)
						: null;
		intakeRightSensor =
				RIGHT_MOTOR_ENABLED
						? intakeMotorRight.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen)
						: null;

		// sensor must be true for 0.1 seconds before being actually true
		intakeFrontSensorDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
		intakeRightSensorDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
		intakeLeftSensorDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

		feederSensorDebouncer = new Debouncer(0.01, Debouncer.DebounceType.kBoth);
		feederSensorIRDebouncer = new Debouncer(0.01, Debouncer.DebounceType.kBoth);

		resetMotors();

		initShuffleboard();
	}

	private void configureMotor(
			CANSparkBase motor, int currentLimit, boolean invert, boolean enableLimitSwitch) {
		motor.restoreFactoryDefaults();
		motor.setIdleMode(IdleMode.kBrake);
		motor.setSmartCurrentLimit(currentLimit);
		motor.setInverted(invert);
		motor
				.getForwardLimitSwitch(com.revrobotics.SparkLimitSwitch.Type.kNormallyOpen)
				.enableLimitSwitch(enableLimitSwitch);
		motor.burnFlash();
	}

	private void configureMotor(CANSparkBase motor, int currentLimit, boolean invert) {
		configureMotor(motor, currentLimit, invert, false);
	}

	private void configureMotor(CANSparkBase motor, boolean invert) {
		configureMotor(motor, 30, invert);
	}

	private void resetMotors() {
		if (FRONT_MOTOR_ENABLED) {
			configureMotor(intakeMotorFront, 25, true);
		}
		if (LEFT_MOTOR_ENABLED) {
			configureMotor(intakeMotorLeft, 25, true);
		}
		if (RIGHT_MOTOR_ENABLED) {
			configureMotor(intakeMotorRight, 25, true);
		}

		configureMotor(ingestMotor, true);
		configureMotor(indexMotorUpper, 40, false);

		configureMotor(feederMotor, 40, true);

		indexMotorUpper.burnFlash();
	}

	public void intakeSet(double speed) {
		if (enableFrontAndSideIntakes) {
			if (FRONT_MOTOR_ENABLED) {
				intakeMotorFront.set(speed);
			}
			if (LEFT_MOTOR_ENABLED) {
				intakeMotorLeft.set(speed);
			}
			if (RIGHT_MOTOR_ENABLED) {
				intakeMotorRight.set(speed);
			}
		}
		ingestMotor.set(speed);
	}

	// intake methods
	public void intakeIn() {
		intakeSet(setIntakeInSpeedEntry.getDouble(INTAKE_IN_SPEED));
	}

	public void intakeReverse() {
		intakeSet(INTAKE_REVERSE_SPEED);
	}

	public void intakeSteal() {

		if (enableFrontAndSideIntakes) {
			if (LEFT_MOTOR_ENABLED) {
				intakeMotorLeft.set(INTAKE_IN_SPEED);
			}
			if (RIGHT_MOTOR_ENABLED) {
				intakeMotorRight.set(INTAKE_IN_SPEED);
			}
			if (FRONT_MOTOR_ENABLED) {
				intakeMotorFront.set(INTAKE_REJECT_SPEED);
			}
		}
		ingestMotor.set(INTAKE_REJECT_SPEED);
		indexMotorUpper.set(INTAKE_IN_SPEED);
	}

	// intake stop methods
	public void intakeStop() {
		intakeSet(0);
	}

	public void intakeFrontStop() {
		if (FRONT_MOTOR_ENABLED) {
			intakeMotorFront.set(0);
		}
	}

	public void intakeLeftStop() {
		if (LEFT_MOTOR_ENABLED) {
			intakeMotorLeft.set(0);
		}
	}

	public void intakeRightStop() {
		if (RIGHT_MOTOR_ENABLED) {
			intakeMotorRight.set(0);
		}
	}

	// intake reject methods
	public void intakeReject() {
		intakeSet(INTAKE_REJECT_SPEED);
	}

	public void intakeFrontReject() {
		if (FRONT_MOTOR_ENABLED) {
			intakeMotorFront.set(INTAKE_REJECT_SPEED);
		}
	}

	public void intakeLeftReject() {
		if (LEFT_MOTOR_ENABLED) {
			intakeMotorLeft.set(INTAKE_REJECT_SPEED);
		}
	}

	public void intakeRightReject() {
		if (RIGHT_MOTOR_ENABLED) {
			intakeMotorRight.set(INTAKE_REJECT_SPEED);
		}
	}

	// index methods
	public void indexIn() {
		indexMotorUpper.set(setIndexInSpeedEntry.getDouble(INDEX_UPPER_IN_SPEED));
		ingestMotor.set(INGEST_LOWER_IN_SPEED);
	}

	public void indexReverse() {
		indexMotorUpper.set(INDEX_UPPER_REVERSE_SPEED);
	}

	public void indexStop() {
		indexMotorUpper.set(0);
	}

	// feeder methods
	public void feederIn() {
		feederMotor.set(setFeederInSpeedEntry.getDouble(FEEDER_IN_SPEED));
	}

	public void feederReverse() {
		feederMotor.set(FEEDER_REVERSE_SPEED);
	}

	public void feederStop() {
		feederMotor.set(0);
	}

	public void feederShoot() {
		feederMotor.set(FEEDER_SHOOT_SPEED);
	}

	// sensor methods
	public boolean indexSensorHasNote() {
		return indexSensor.isPressed();
	}

	public boolean debouncedFeederSensor() {
		// return feederSensorDebouncer.calculate(feederSensor.isPressed());

		boolean feederSensorSignal = feederSensorDebouncer.calculate(feederSensor.isPressed());
		boolean feederSensorIRSignal = feederSensorIRDebouncer.calculate(!feederSensorIR.get());

		return feederSensorSignal || feederSensorIRSignal;
	}

	public boolean feederSensorHasNote() {
		return debouncedFeederSensor() && !getSensorOverride();
	}

	public boolean intakeFrontSeesNote() {
		return false;
		// return intakeFrontSensor.isPressed();
	}

	public boolean intakeLeftSeesNote() {
		return intakeLeftSensor.isPressed();
	}

	public boolean intakeRightSeesNote() {
		return intakeRightSensor.isPressed();
	}

	// debounce sensors
	public boolean debouncedIntakeFrontSensor() {
		// if (intakeFrontSensorDebouncer.calculate(intakeFrontSensor.isPressed())) {
		// 	return true;
		// }
		return false;
		// return intakeFrontSensorDebouncer.calculate(intakeFrontSensor.isPressed());
	}

	public boolean debouncedIntakeLeftSensor() {
		// if (intakeLeftSensorDebouncer.calculate(intakeLeftSensor.isPressed())) {
		// 	return true;
		// }
		return false;
		// return intakeLeftSensorDebouncer.calculate(intakeFrontSensor.isPressed());
	}

	public boolean debouncedIntakeRightSensor() {
		// if (intakeRightSensorDebouncer.calculate(intakeRightSensor.isPressed())) {
		// 	return true;
		// }
		return false;
		// return intakeRightSensorDebouncer.calculate(intakeFrontSensor.isPressed());
	}

	// override methods on shuffleboard
	public boolean getSensorOverride() {
		return sensorOverride.getBoolean(false);
	}

	public boolean getRejectOverride() {
		return rejectOverride.getBoolean(false);
	}

	public boolean isIntakeOn() {
		return ((LEFT_MOTOR_ENABLED && intakeMotorLeft.get() > 0)
				|| indexMotorUpper.get() > 0
				|| ingestMotor.get() > 0
				|| feederMotor.get() > 0);
	}

	public boolean isLeftIntakeRunning() {
		return Math.abs(intakeMotorLeft.getEncoder().getVelocity()) > 1.0;
	}

	public boolean isRightIntakeRunning() {
		return Math.abs(intakeMotorRight.getEncoder().getVelocity()) > 1.0;
	}

	public boolean isIndexRunning() {
		return Math.abs(indexMotorUpper.getEncoder().getVelocity()) > 1.0;
	}

	// logging
	public void initShuffleboard() {
		if (Robot.isDebugMode()) {
			shuffleboardTab
					.addDouble(
							"Front Intake Motor Temp",
							() -> FRONT_MOTOR_ENABLED ? intakeMotorFront.getMotorTemperature() : -1)
					.withSize(1, 1)
					.withWidget(BuiltInWidgets.kTextView);
			shuffleboardTab
					.addDouble(
							"Left Intake Motor Temp",
							() -> LEFT_MOTOR_ENABLED ? intakeMotorLeft.getMotorTemperature() : -1)
					.withSize(1, 1)
					.withWidget(BuiltInWidgets.kTextView);
			shuffleboardTab
					.addDouble(
							"Right Intake Motor Temp",
							() -> RIGHT_MOTOR_ENABLED ? intakeMotorRight.getMotorTemperature() : -1)
					.withSize(1, 1)
					.withWidget(BuiltInWidgets.kTextView);
			shuffleboardTab
					.addDouble("Ingest Motor Temp", () -> ingestMotor.getMotorTemperature())
					.withSize(1, 1)
					.withWidget(BuiltInWidgets.kTextView);
			shuffleboardTab
					.addDouble("Index Motor Temp", () -> indexMotorUpper.getMotorTemperature())
					.withSize(1, 1)
					.withWidget(BuiltInWidgets.kTextView);
			shuffleboardTab
					.addDouble("Feeder Motor Temp", () -> feederMotor.getMotorTemperature())
					.withSize(1, 1)
					.withWidget(BuiltInWidgets.kTextView);
		}

		sensorOverride =
				Shuffleboard.getTab("Intake")
						.add("Override Sensors", false)
						.withSize(1, 1)
						.withWidget(BuiltInWidgets.kToggleSwitch)
						.getEntry();

		shuffleboardTab.addBoolean("Index Sensor - ", this::indexSensorHasNote).withSize(1, 1);
		shuffleboardTab.addBoolean("Feeder Sensor - ", this::feederSensorHasNote).withSize(1, 1);

		shuffleboardTab.addBoolean(
				"Feeder beambreak", () -> feederSensorDebouncer.calculate(feederSensor.isPressed()));
		shuffleboardTab.addBoolean(
				"Feeder IR", () -> feederSensorIRDebouncer.calculate(!feederSensorIR.get()));

		// no intake back sensor
		shuffleboardTab.addBoolean("Intake Front Sensor - ", this::intakeFrontSeesNote).withSize(1, 1);
		shuffleboardTab.addBoolean("Intake Left Sensor - ", this::intakeLeftSeesNote).withSize(1, 1);
		shuffleboardTab.addBoolean("Intake Right Sensor - ", this::intakeRightSeesNote).withSize(1, 1);

		setIntakeInSpeedEntry =
				shuffleboardTab
						.addPersistent("Intake in speed - ", INTAKE_IN_SPEED)
						.withSize(2, 1)
						.withProperties(Map.of("Min", -1, "Max", 1))
						.getEntry();

		setIndexInSpeedEntry =
				shuffleboardTab.add("Index in speed - ", INDEX_UPPER_IN_SPEED).withSize(1, 1).getEntry();

		setFeederInSpeedEntry =
				shuffleboardTab.add("Feeder in speed - ", FEEDER_IN_SPEED).withSize(1, 1).getEntry();

		rejectOverride =
				Shuffleboard.getTab("Intake")
						.add("Override Intake Reject", false)
						.withSize(1, 1)
						.withWidget(BuiltInWidgets.kToggleSwitch)
						.getEntry();
	}

	// auto thing

	/*
	 * returns true if all the motors are set to be not moving
	 */
	public boolean isIntakeRunning() {
		return (!LEFT_MOTOR_ENABLED || intakeMotorLeft.get() != 0)
				&& (!RIGHT_MOTOR_ENABLED || intakeMotorRight.get() != 0);
	}
}
