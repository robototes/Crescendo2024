package frc.team2412.robot.subsystems;

import static frc.team2412.robot.Hardware.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
	public static final double INTAKE_OUT_SPEED = -0.7;
	public static final double INTAKE_IN_SPEED = 0.3;

	public static final double INDEX_IN_SPEED = 0.3;
	public static final double INDEX_OUT_SPEED = 0.3;

	public static final double FEEDER_IN_SPEED = 0.3;
	public static final double FEEDER_OUT_SPEED = 0.3;

	// 'skirt' intake
	private final CANSparkFlex intakeMotorFront;
	private final CANSparkFlex intakeMotorBack;
	private final CANSparkFlex intakeMotorLeft;
	private final CANSparkFlex intakeMotorRight;

	private final CANSparkFlex indexMotorTop;
	private final CANSparkFlex indexMotorBottom;

	private final CANSparkFlex feederMotor;

	public IntakeSubsystem() {

		intakeMotorFront = new CANSparkFlex(INTAKE_MOTOR_FRONT, MotorType.kBrushless);
		intakeMotorBack = new CANSparkFlex(INTAKE_MOTOR_BACK, MotorType.kBrushless);
		intakeMotorLeft = new CANSparkFlex(INTAKE_MOTOR_LEFT, MotorType.kBrushless);
		intakeMotorRight = new CANSparkFlex(INTAKE_MOTOR_RIGHT, MotorType.kBrushless);

		indexMotorTop = new CANSparkFlex(INDEX_MOTOR_TOP, MotorType.kBrushless);
		indexMotorBottom = new CANSparkFlex(INDEX_MOTOR_BOTTOM, MotorType.kBrushless);

		feederMotor = new CANSparkFlex(FEEDER_MOTOR, MotorType.kBrushless);

		resetMotors();
	}

	public void resetMotors() {
		// reset
		intakeMotorFront.restoreFactoryDefaults();
		intakeMotorBack.restoreFactoryDefaults();
		intakeMotorLeft.restoreFactoryDefaults();
		intakeMotorRight.restoreFactoryDefaults();

		indexMotorTop.restoreFactoryDefaults();
		indexMotorBottom.restoreFactoryDefaults();

		feederMotor.restoreFactoryDefaults();

		// brake motors
		intakeMotorFront.setIdleMode(IdleMode.kBrake);
		intakeMotorBack.setIdleMode(IdleMode.kBrake);
		intakeMotorLeft.setIdleMode(IdleMode.kBrake);
		intakeMotorRight.setIdleMode(IdleMode.kBrake);

		indexMotorTop.setIdleMode(IdleMode.kBrake);
		indexMotorBottom.setIdleMode(IdleMode.kBrake);

		feederMotor.setIdleMode(IdleMode.kBrake);

		// Invert motors
		intakeMotorFront.setInverted(true);
		intakeMotorBack.setInverted(true);
		intakeMotorLeft.setInverted(true);
		intakeMotorRight.setInverted(true);

		indexMotorTop.setInverted(true);
		indexMotorBottom.setInverted(true);

		// Limit voltage for motors
		intakeMotorFront.setSmartCurrentLimit(20);
		intakeMotorBack.setSmartCurrentLimit(20);
		intakeMotorLeft.setSmartCurrentLimit(20);
		intakeMotorRight.setSmartCurrentLimit(20);

		indexMotorTop.setSmartCurrentLimit(20);
		indexMotorBottom.setSmartCurrentLimit(20);

		feederMotor.setSmartCurrentLimit(20);

		// save motor settings even while motor dies
		intakeMotorFront.burnFlash();
		intakeMotorBack.burnFlash();
		intakeMotorLeft.burnFlash();
		intakeMotorRight.burnFlash();

		indexMotorTop.burnFlash();
		indexMotorBottom.burnFlash();

		feederMotor.burnFlash();
	}

	// intake methods
	public void intakeIn() {
		intakeMotorFront.set(INTAKE_IN_SPEED);
		intakeMotorLeft.set(INTAKE_IN_SPEED);
		intakeMotorBack.set(INTAKE_IN_SPEED);
		intakeMotorFront.set(INTAKE_IN_SPEED);
	}

	public void intakeOut() {
		intakeMotorFront.set(INTAKE_OUT_SPEED);
		intakeMotorBack.set(INTAKE_OUT_SPEED);
		intakeMotorLeft.set(INTAKE_OUT_SPEED);
		intakeMotorRight.set(INTAKE_OUT_SPEED);
	}

	public void stopIntakeMotors() {
		intakeMotorFront.set(0);
		intakeMotorBack.set(0);
		intakeMotorLeft.set(0);
		intakeMotorRight.set(0);
	}

	// index methods
	public void indexIn() {
		indexMotorTop.set(INDEX_IN_SPEED);
		indexMotorBottom.set(INDEX_IN_SPEED);
	}

	public void indexOut() {
		indexMotorTop.set(INDEX_OUT_SPEED);
		indexMotorBottom.set(INDEX_OUT_SPEED);
	}

	public void stopIndexMotors() {
		indexMotorTop.set(0);
		indexMotorBottom.set(0);
	}

	// feeder methods
	public void feederIn() {
		feederMotor.set(FEEDER_IN_SPEED);
	}

	public void feederOut() {
		feederMotor.set(FEEDER_OUT_SPEED);
	}

	public void stopFeeder() {
		feederMotor.set(0);
	}
}
