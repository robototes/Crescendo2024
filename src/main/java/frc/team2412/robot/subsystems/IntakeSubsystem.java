package frc.team2412.robot.subsystems;

import static frc.team2412.robot.Hardware.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;

public class IntakeSubsystem extends SubsystemBase {
	// Constants
	public static final double INTAKE_IN_SPEED = 0.3;
	public static final double INTAKE_OUT_SPEED = -0.7;

	public static final double INDEX_IN_SPEED = 0.3;
	public static final double INDEX_OUT_SPEED = -0.3;

	public static final double FEEDER_IN_SPEED = 0.3;
	public static final double FEEDER_OUT_SPEED = -0.3;

	// Motors
	private final CANSparkMax intakeMotorFront;
	private final CANSparkMax intakeMotorBack;
	private final CANSparkMax intakeMotorLeft;
	private final CANSparkMax intakeMotorRight;

	private final CANSparkFlex indexMotor;

	private final CANSparkFlex feederMotor;

	// Shuffleboard
	private final GenericEntry setIntakeSpeedEntry =
			Shuffleboard.getTab("Intake")
					.addPersistent("Intake speed - ", INTAKE_IN_SPEED)
					.withSize(2, 1)
					.withProperties(Map.of("Min", -1, "Max", 1))
					.getEntry();

	private final GenericEntry setIndexSpeedEntry =
			Shuffleboard.getTab("Intake").add("Index speed - ", INDEX_IN_SPEED).withSize(1, 1).getEntry();

	private final GenericEntry setFeederSpeedEntry =
			Shuffleboard.getTab("Intake")
					.add("Feeder Speed - ", FEEDER_IN_SPEED)
					.withSize(1, 1)
					.getEntry();

	public IntakeSubsystem() {
		intakeMotorFront = new CANSparkMax(INTAKE_MOTOR_FRONT, MotorType.kBrushless);
		intakeMotorBack = new CANSparkMax(INTAKE_MOTOR_BACK, MotorType.kBrushless);
		intakeMotorLeft = new CANSparkMax(INTAKE_MOTOR_LEFT, MotorType.kBrushless);
		intakeMotorRight = new CANSparkMax(INTAKE_MOTOR_RIGHT, MotorType.kBrushless);

		indexMotor = new CANSparkFlex(INDEX_MOTOR, MotorType.kBrushless);

		feederMotor = new CANSparkFlex(FEEDER_MOTOR, MotorType.kBrushless);

		resetMotors();
	}

	// commented out since we have two motor types now
	//private void configureMotor(CANSparkFlex motor) {
	//	motor.restoreFactoryDefaults();
	//	motor.setIdleMode(IdleMode.kBrake);
	//	motor.setSmartCurrentLimit(20);
	//	motor.burnFlash();
	//}

	private void resetMotors() {
		intakeMotorFront.restoreFactoryDefaults();
		intakeMotorFront.setIdleMode(IdleMode.kBrake);
		intakeMotorFront.setSmartCurrentLimit(20);
		intakeMotorFront.burnFlash();
		
		intakeMotorBack.restoreFactoryDefaults();
		intakeMotorBack.setIdleMode(IdleMode.kBrake);
		intakeMotorBack.setSmartCurrentLimit(20);
		intakeMotorBack.burnFlash();
		intakeMotorBack.follow(intakeMotorFront);

		intakeMotorLeft.restoreFactoryDefaults();
		intakeMotorLeft.setIdleMode(IdleMode.kBrake);
		intakeMotorLeft.setSmartCurrentLimit(20);
		intakeMotorLeft.burnFlash();
		intakeMotorLeft.follow(intakeMotorFront);

		intakeMotorRight.restoreFactoryDefaults();
		intakeMotorRight.setIdleMode(IdleMode.kBrake);
		intakeMotorRight.setSmartCurrentLimit(20);
		intakeMotorRight.burnFlash();
		intakeMotorRight.follow(intakeMotorFront);

		indexMotor.restoreFactoryDefaults();
		indexMotor.setIdleMode(IdleMode.kBrake);
		indexMotor.setSmartCurrentLimit(20);
		indexMotor.burnFlash();
		
		feederMotor.restoreFactoryDefaults();
		feederMotor.setIdleMode(IdleMode.kBrake);
		feederMotor.setSmartCurrentLimit(20);
		feederMotor.burnFlash();
	}

	// intake methods
	public void intakeIn() {
		intakeMotorFront.set(INTAKE_IN_SPEED);
	}

	public void intakeOut() {
		intakeMotorFront.set(INTAKE_OUT_SPEED);
	}

	public void intakeStop() {
		intakeMotorFront.set(0);
	}

	// index methods
	public void indexIn() {
		indexMotor.set(INDEX_IN_SPEED);
	}

	public void indexOut() {
		indexMotor.set(INDEX_OUT_SPEED);
	}

	public void indexStop() {
		indexMotor.set(0);
	}

	// feeder methods
	public void feederIn() {
		feederMotor.set(FEEDER_IN_SPEED);
	}

	public void feederOut() {
		feederMotor.set(FEEDER_OUT_SPEED);
	}

	public void feederStop() {
		feederMotor.set(0);
	}
}
