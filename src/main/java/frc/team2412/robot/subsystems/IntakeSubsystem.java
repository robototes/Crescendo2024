package frc.team2412.robot.subsystems;

import static frc.team2412.robot.Hardware.*;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;

public class IntakeSubsystem extends SubsystemBase {
	// Constants
	public static final double INTAKE_IN_SPEED = 0.3;
	public static final double INTAKE_REVERSE_SPEED = -0.7;
	public static final double INTAKE_REJECT_SPEED = -0.4;

	public static final double INDEX_UPPER_IN_SPEED = 0.3;
	public static final double INDEX_UPPER_REVERSE_SPEED = -0.3;

	public static final double INJEST_SPEED = 0.3;
	public static final double INJEST_REVERSE_SPEED = -0.3;

	public static final double FEEDER_SHOOT_SPEED = 1.0;

	public static final double FEEDER_IN_SPEED = 0.65;
	public static final double FEEDER_REVERSE_SPEED = -0.3;

	// Motors
	private final CANSparkMax intakeMotorFront;
	private final CANSparkMax intakeMotorBack;
	private final CANSparkMax intakeMotorLeft;
	private final CANSparkMax intakeMotorRight;

	private final CANSparkFlex injestMotor;
	private final CANSparkFlex indexMotorUpper;

	private final CANSparkFlex feederMotor;

	// Sensors
	private final DigitalInput indexSensor;
	private final DigitalInput feederSensor;

	// Shuffleboard
	// speed
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

	// temperature
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

	private final GenericEntry injestMotorTemp =
			Shuffleboard.getTab("Intake")
					.add("Injest temp", 0)
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
					.withWidget(BuiltInWidgets.kTextView)
					.getEntry();

	public IntakeSubsystem() {
		intakeMotorFront = new CANSparkMax(INTAKE_MOTOR_FRONT, MotorType.kBrushless);
		intakeMotorBack = new CANSparkMax(INTAKE_MOTOR_BACK, MotorType.kBrushless);
		intakeMotorLeft = new CANSparkMax(INTAKE_MOTOR_LEFT, MotorType.kBrushless);
		intakeMotorRight = new CANSparkMax(INTAKE_MOTOR_RIGHT, MotorType.kBrushless);

		injestMotor = new CANSparkFlex(INJEST_MOTOR, MotorType.kBrushless);
		indexMotorUpper = new CANSparkFlex(INDEX_MOTOR_UPPER, MotorType.kBrushless);

		feederMotor = new CANSparkFlex(FEEDER_MOTOR, MotorType.kBrushless);

		indexSensor = new DigitalInput(INDEX_SENSOR);
		feederSensor = new DigitalInput(FEEDER_SENSOR);

		resetMotors();

		ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Intake");
		shuffleboardTab.addBoolean("Index Sensor - ", indexSensor::get).withSize(1, 1);
		shuffleboardTab.addBoolean("Feeder Sensor - ", feederSensor::get).withSize(1, 1);
	}

	private void configureMotor(CANSparkBase motor) {
		motor.restoreFactoryDefaults();
		motor.setIdleMode(IdleMode.kBrake);
		motor.setSmartCurrentLimit(20);
		motor.burnFlash();
	}

	private void resetMotors() {
		configureMotor(intakeMotorFront);
		configureMotor(intakeMotorBack);
		configureMotor(intakeMotorLeft);
		configureMotor(intakeMotorRight);

		configureMotor(injestMotor);
		configureMotor(indexMotorUpper);

		configureMotor(feederMotor);
	}

	public void intakeSet(double speed) {
		intakeMotorFront.set(speed);
		intakeMotorLeft.set(speed);
		intakeMotorRight.set(speed);
		intakeMotorBack.set(speed);

		injestMotor.set(speed);
	}

	// intake methods
	public void intakeIn() {
		intakeSet(setIntakeInSpeedEntry.getDouble(INTAKE_IN_SPEED));
	}

	public void intakeReverse() {
		intakeSet(INTAKE_REVERSE_SPEED);
	}

	public void intakeStop() {
		intakeSet(0);
	}

	public void intakeReject() {
		intakeSet(INTAKE_REJECT_SPEED);
	}

	// index methods
	public void indexIn() {
		indexMotorUpper.set(setIndexInSpeedEntry.getDouble(INDEX_UPPER_IN_SPEED));
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

	public boolean getIndexSensor() {
		return indexSensor.get() && !getSensorOverride();
	}

	public boolean getFeederSensor() {
		return feederSensor.get() && !getSensorOverride();
	}

	public boolean getSensorOverride() {
		return sensorOverride.getBoolean(false);
	}

	@Override
	public void periodic() {
		intakeMotorFrontTemp.setDouble(intakeMotorFront.getMotorTemperature());
		intakeMotorBackTemp.setDouble(intakeMotorBack.getMotorTemperature());
		intakeMotorRightTemp.setDouble(intakeMotorRight.getMotorTemperature());
		intakeMotorLeftTemp.setDouble(intakeMotorLeft.getMotorTemperature());

		injestMotorTemp.setDouble(injestMotor.getMotorTemperature());

		indexMotorUpperTemp.setDouble(indexMotorUpper.getMotorTemperature());

		feederMotorTemp.setDouble(feederMotor.getMotorTemperature());
	}
}
