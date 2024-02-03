package frc.team2412.robot.subsystems;

import static frc.team2412.robot.Hardware.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import java.util.Map;

public class IntakeSubsystem extends SubsystemBase {
	public static final double INTAKE_OUT_SPEED = -0.7;
	public static final double INTAKE_IN_SPEED = 0.3;
	public static final double INGEST_SPEED = 0.2;
	public static final double INDEX_SPEED = 0.2;
	public static final double FEEDER_SPEED = 0.25;

	//Hardware
	private final CANSparkFlex intakeMotorFront;
	private final CANSparkFlex intakeMotorBack;
	private final CANSparkFlex intakeMotorLeft;
	private final CANSparkFlex intakeMotorRight;


	private final CANSparkFlex indexMotorTop;
	private final CANSparkFlex indexMotorBottom;

	private final CANSparkFlex feederMotor;



	//shuffleboard :O
	private final GenericEntry intakeSpeedEntry =
			Shuffleboard.getTab("Intake")
					.addPersistent("Intake Speed", INTAKE_IN_SPEED)
					.withSize(1, 1)
					.withWidget(BuiltInWidgets.kNumberSlider)
					.withProperties(Map.of("Min", -1, "Max", 1))
					.getEntry();


	//CONSTRUCTOR
	public IntakeSubsystem() {

		intakeMotorFront = new CANSparkFlex(INTAKE_MOTOR_FRONT, MotorType.kBrushless);
		intakeMotorBack = new CANSparkFlex(INTAKE_MOTOR_BACK, MotorType.kBrushless);
		intakeMotorLeft = new CANSparkFlex(INTAKE_MOTOR_LEFT, MotorType.kBrushless);
		intakeMotorRight = new CANSparkFlex(INTAKE_MOTOR_RIGHT, MotorType.kBrushless);

		indexMotorTop = new CANSparkFlex(INDEX_MOTOR_TOP, MotorType.kBrushless);
		indexMotorBottom = new CANSparkFlex(INDEX_MOTOR_BOTTOM, MotorType.kBrushless);

		feederMotor = new CANSparkFlex(FEEDER_MOTOR, MotorType.kBrushless);
	}

	//METHODS

	//Configuring Motors
	private void configureMotor(CANSparkFlex motor) {
		motor.restoreFactoryDefaults();
		motor.setIdleMode(IdleMode.kBrake);
		motor.setInverted(true);
		motor.setSmartCurrentLimit(20);
		motor.burnFlash();
	}
	public void resetMotors() {
		configureMotor(intakeMotorFront);
		configureMotor(intakeMotorBack);
		configureMotor(intakeMotorLeft);
		configureMotor(intakeMotorRight);

		configureMotor(indexMotorTop);
		configureMotor(indexMotorBottom);
		
		configureMotor(feederMotor);
	}


	//MOTOR METHODS

	//intake motor methods
	private void intake(CANSparkFlex motor) {
			motor.set(INTAKE_IN_SPEED);
		}
	private void index(CANSparkFlex motor) {
		motor.set(INDEX_SPEED);
	}
	private void feeder(CANSparkFlex motor) {
		motor.set(FEEDER_SPEED);
	}

	public void intakeIn() {
		intake(intakeMotorFront);
		intake(intakeMotorBack);
		intake(intakeMotorLeft);
		intake(intakeMotorRight);
	}

	public void indexIn() {
		index(indexMotorTop);
		index(indexMotorBottom);
	}

	public void feederIn() {
		feeder(feederMotor);
	}
	


	//Stop Motor methods
	private void stopMotor(CANSparkFlex motor) {
		motor.set(0);
	}

	public void stopIntakeMotors() {
		stopMotor(intakeMotorFront);
        stopMotor(intakeMotorBack);
        stopMotor(intakeMotorLeft);
        stopMotor(intakeMotorRight);
	}

	public void stopIndexMotor() {
		stopMotor(indexMotorTop);
		stopMotor(indexMotorBottom);
	}

	public void stopFeederMotor() {
		stopMotor(feederMotor);
	}
	
}