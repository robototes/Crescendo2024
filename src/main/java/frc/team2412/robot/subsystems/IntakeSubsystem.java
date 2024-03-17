package frc.team2412.robot.subsystems;

import static frc.team2412.robot.Hardware.*;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;

public class IntakeSubsystem extends SubsystemBase {
	// Constants
	public static final double INTAKE_IN_SPEED = 0.6;
	public static final double INTAKE_REVERSE_SPEED = -0.7;
	public static final double INTAKE_REJECT_SPEED = -0.4;

	public static final double INDEX_UPPER_IN_SPEED = 1.0;
	public static final double INDEX_UPPER_REVERSE_SPEED = -0.3;

	public static final double INGEST_LOWER_IN_SPEED = 0.8;
	public static final double INGEST_LOWER_REVERSE_SPEED = -0.3;

	public static final double FEEDER_SHOOT_SPEED = 1.0;

	public static final double FEEDER_IN_SPEED = 0.65;
	public static final double FEEDER_REVERSE_SPEED = -0.3;

	// Motors
	private final CANSparkMax intakeMotorFront;
	private final CANSparkMax intakeMotorBack;
	private final CANSparkMax intakeMotorLeft;
	private final CANSparkMax intakeMotorRight;

	private final CANSparkFlex ingestMotor;
	private final CANSparkFlex indexMotorUpper;

	private final CANSparkFlex feederMotor;

	// Sensors
	private final DigitalInput indexSensor;
	private final DigitalInput feederSensor;

	private final SparkLimitSwitch intakeFrontSensor;
	// private final SparkLimitSwitch intakeBackSensor;
	private final SparkLimitSwitch intakeLeftSensor;
	private final SparkLimitSwitch intakeRightSensor;

	// Shuffleboard
	// control intake speed
	private final GenericEntry setIntakeInSpeedEntry =
			Shuffleboard.getTab("Intake")
					.addPersistent("Intake in speed - ", INTAKE_IN_SPEED)
					.withSize(2, 1)
					.withProperties(Map.of("Min", -1, "Max", 1))
					.getEntry();

	private final GenericEntry setIndexInSpeedEntry =
			Shuffleboard.getTab("Intake")
					.add("Index in speed - ", INDEX_UPPER_IN_SPEED)
					.withSize(1, 1)
					.getEntry();

	private final GenericEntry setFeederInSpeedEntry =
			Shuffleboard.getTab("Intake")
					.add("Feeder in speed - ", FEEDER_IN_SPEED)
					.withSize(1, 1)
					.getEntry();

	// motor temperature
	private final GenericEntry intakeMotorFrontTemp =
			Shuffleboard.getTab("Intake")
					.add("Front Intake temp", 0)
					.withSize(1, 1)
					.withWidget(BuiltInWidgets.kTextView)
					.getEntry();

	private final GenericEntry intakeMotorBackTemp =
			Shuffleboard.getTab("Intake")
					.add("Back Intake temp", 0)
					.withSize(1, 1)
					.withWidget(BuiltInWidgets.kTextView)
					.getEntry();

	private final GenericEntry intakeMotorLeftTemp =
			Shuffleboard.getTab("Intake")
					.add("Left Intake temp", 0)
					.withSize(1, 1)
					.withWidget(BuiltInWidgets.kTextView)
					.getEntry();

	private final GenericEntry intakeMotorRightTemp =
			Shuffleboard.getTab("Intake")
					.add("Right Intake temp", 0)
					.withSize(1, 1)
					.withWidget(BuiltInWidgets.kTextView)
					.getEntry();

	private final GenericEntry indexMotorUpperTemp =
			Shuffleboard.getTab("Intake")
					.add("Upper Index temp", 0)
					.withSize(1, 1)
					.withWidget(BuiltInWidgets.kTextView)
					.getEntry();

	private final GenericEntry ingestMotorTemp =
			Shuffleboard.getTab("Intake")
					.add("Ingest temp", 0)
					.withSize(1, 1)
					.withWidget(BuiltInWidgets.kTextView)
					.getEntry();

	private final GenericEntry feederMotorTemp =
			Shuffleboard.getTab("Intake")
					.add("Feeder temp", 0)
					.withSize(1, 1)
					.withWidget(BuiltInWidgets.kTextView)
					.getEntry();

	// sensor override
	private final GenericEntry sensorOverride =
			Shuffleboard.getTab("Intake")
					.add("Override Sensors", false)
					.withSize(1, 1)
					.withWidget(BuiltInWidgets.kToggleSwitch)
					.getEntry();

	// reject override
	private final GenericEntry rejectOverride =
			Shuffleboard.getTab("Intake")
					.add("Override Intake Reject", false)
					.withSize(1, 1)
					.withWidget(BuiltInWidgets.kToggleSwitch)
					.getEntry();

	public IntakeSubsystem() {

		intakeMotorFront = new CANSparkMax(INTAKE_MOTOR_FRONT, MotorType.kBrushless);
		intakeMotorBack = new CANSparkMax(INTAKE_MOTOR_BACK, MotorType.kBrushless);
		intakeMotorLeft = new CANSparkMax(INTAKE_MOTOR_LEFT, MotorType.kBrushless);
		intakeMotorRight = new CANSparkMax(INTAKE_MOTOR_RIGHT, MotorType.kBrushless);

		ingestMotor = new CANSparkFlex(INGEST_MOTOR, MotorType.kBrushless);
		indexMotorUpper = new CANSparkFlex(INDEX_MOTOR_UPPER, MotorType.kBrushless);

		feederMotor = new CANSparkFlex(FEEDER_MOTOR, MotorType.kBrushless);

		indexSensor = new DigitalInput(INDEX_SENSOR);
		feederSensor = new DigitalInput(FEEDER_SENSOR);

		intakeFrontSensor = intakeMotorFront.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
		// intakeBackSensor =
		// intakeMotorBack.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
		intakeLeftSensor = intakeMotorLeft.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
		intakeRightSensor = intakeMotorRight.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

		intakeFrontSensor.enableLimitSwitch(false);
		intakeLeftSensor.enableLimitSwitch(false);
		intakeRightSensor.enableLimitSwitch(false);

		resetMotors();

		// show if sensors have notes in shuffleboard
		ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Intake");
		shuffleboardTab.addBoolean("Index Sensor - ", this::indexSensorHasNote).withSize(1, 1);
		shuffleboardTab.addBoolean("Feeder Sensor - ", this::feederSensorHasNote).withSize(1, 1);
		// intake back sensor
		shuffleboardTab.addBoolean("Intake Front Sensor - ", this::intakeFrontSeesNote).withSize(1, 1);
		shuffleboardTab.addBoolean("Intake Left Sensor - ", this::intakeLeftSeesNote).withSize(1, 1);
		shuffleboardTab.addBoolean("Intake Right Sensor - ", this::intakeRightSeesNote).withSize(1, 1);
	}

	private void configureMotor(CANSparkBase motor, int currentLimit, boolean invert) {
		motor.restoreFactoryDefaults();
		motor.setIdleMode(IdleMode.kBrake);
		motor.setSmartCurrentLimit(currentLimit);
		motor.setInverted(invert);
		motor.burnFlash();
	}

	private void configureMotor(CANSparkBase motor, boolean invert) {
		configureMotor(motor, 20, invert);
	}

	private void resetMotors() {
		configureMotor(intakeMotorFront, true);
		configureMotor(intakeMotorBack, true);
		configureMotor(intakeMotorLeft, true);
		configureMotor(intakeMotorRight, true);

		configureMotor(ingestMotor, false);
		configureMotor(indexMotorUpper, 40, true);

		configureMotor(feederMotor, 40, true);
	}

	public void intakeSet(double speed) {
		intakeMotorFront.set(speed);
		intakeMotorLeft.set(speed);
		intakeMotorRight.set(speed);
		intakeMotorBack.set(speed);

		ingestMotor.set(speed);
	}

	// intake methods
	public void intakeIn() {
		intakeSet(setIntakeInSpeedEntry.getDouble(INTAKE_IN_SPEED));
	}

	public void intakeReverse() {
		intakeSet(INTAKE_REVERSE_SPEED);
	}

	// intake stop methods
	public void intakeStop() {
		intakeSet(0);
	}

	public void intakeFrontStop() {
		intakeMotorFront.set(0);
	}

	public void intakeBackStop() {
		intakeMotorBack.set(0);
	}

	public void intakeLeftStop() {
		intakeMotorLeft.set(0);
	}

	public void intakeRightStop() {
		intakeMotorRight.set(0);
	}

	// intake reject methods
	public void intakeReject() {
		intakeSet(INTAKE_REJECT_SPEED);
	}

	public void intakeFrontReject() {
		intakeMotorFront.set(INTAKE_REJECT_SPEED);
	}

	public void intakeBackReject() {
		intakeMotorBack.set(INTAKE_REJECT_SPEED);
	}

	public void intakeLeftReject() {
		intakeMotorLeft.set(INTAKE_REJECT_SPEED);
	}

	public void intakeRightReject() {
		intakeMotorRight.set(INTAKE_REJECT_SPEED);
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
		return !indexSensor.get() && !getSensorOverride();
	}

	public boolean feederSensorHasNote() {
		return !feederSensor.get() && !getSensorOverride();
	}

	public boolean intakeFrontSeesNote() {
		return intakeFrontSensor.isPressed();
	}

	public boolean intakeLeftSeesNote() {
		return intakeLeftSensor.isPressed();
	}

	public boolean intakeRightSeesNote() {
		return intakeRightSensor.isPressed();
	}

	// override methods on shuffleboard
	public boolean getSensorOverride() {
		return sensorOverride.getBoolean(false);
	}

	public boolean getRejectOverride() {
		return rejectOverride.getBoolean(false);
	}

	@Override
	public void periodic() {
		intakeMotorFrontTemp.setDouble(intakeMotorFront.getMotorTemperature());
		intakeMotorBackTemp.setDouble(intakeMotorBack.getMotorTemperature());
		intakeMotorRightTemp.setDouble(intakeMotorRight.getMotorTemperature());
		intakeMotorLeftTemp.setDouble(intakeMotorLeft.getMotorTemperature());

		ingestMotorTemp.setDouble(ingestMotor.getMotorTemperature());

		indexMotorUpperTemp.setDouble(indexMotorUpper.getMotorTemperature());

		feederMotorTemp.setDouble(feederMotor.getMotorTemperature());
	}
}
