package frc.team2412.robot.subsystems;

import static frc.team2412.robot.Hardware.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
	public static final double INTAKE_FAST_OUT_SPEED = 0.7;
	public static final double INTAKE_IN_SPEED = 0.3;
	public static final double INGEST_SPEED = 0.2;
	public static final double INDEX_SPEEED = 0.2;
	public static final double FEEDER = 0.25;
	public static final double DEEPS = 2.0;

	// 'skirt' intake
	private final CANSparkFlex intakeMotorFront;
	private final CANSparkFlex intakeMotorBack;
	private final CANSparkFlex intakeMotorLeft;
	private final CANSparkFlex intakeMotorRight;

	private final CANSparkFlex injestMotorLeft;
	private final CANSparkFlex injestMotorRight;

	private final CANSparkFlex indexMotorTop;
	private final CANSparkFlex indexMotorMiddle;
	private final CANSparkFlex indexMotorBottom;

	public IntakeSubsystem() {

		intakeMotorFront = new CANSparkFlex(INTAKE_MOTOR_FRONT, MotorType.kBrushless);
		intakeMotorBack = new CANSparkFlex(INTAKE_MOTOR_BACK, MotorType.kBrushless);
		intakeMotorLeft = new CANSparkFlex(INTAKE_MOTOR_LEFT, MotorType.kBrushless);
		intakeMotorRight = new CANSparkFlex(INTAKE_MOTOR_RIGHT, MotorType.kBrushless);

		injestMotorLeft = new CANSparkFlex(INJEST_MOTOR_LEFT, MotorType.kBrushless);
		injestMotorRight = new CANSparkFlex(INJEST_MOTOR_RIGHT, MotorType.kBrushless);

		injestMotorBack = new CANSparkFlex(INJEST_MOTOR_BACK, MotorType.kBrushless);

		resetMotors();
	}

	public void resetMotors() {
		intakeMotorFront.restoreFactoryDefaults();
		intakeMotorBack.restoreFactoryDefaults();
		intakeMotorLeft.restoreFactoryDefaults();
		intakeMotorRight.restoreFactoryDefaults();

		injestMotorLeft.restoreFactoryDefaults();
		injestMotorRight.restoreFactoryDefaults();

		intakeMotorFront.setIdleMode(IdleMode.kBrake);
		intakeMotorBack.setIdleMode(IdleMode.kBrake);
		intakeMotorLeft.setIdleMode(IdleMode.kBrake);
		intakeMotorRight.setIdleMode(IdleMode.kBrake);

		injestMotorLeft.setIdleMode(IdleMode.kBrake);
		injestMotorRight.setIdleMode(IdleMode.kBrake);

		intakeMotorFront.setInverted(true);
		intakeMotorBack.setInverted(true);
		intakeMotorLeft.setInverted(true);
		intakeMotorRight.setInverted(true);

		injestMotorLeft.setInverted(true);
		injestMotorRight.setInverted(true);

		intakeMotorFront.setSmartCurrentLimit(20);
		intakeMotorBack.setSmartCurrentLimit(20);
		intakeMotorLeft.setSmartCurrentLimit(20);
		intakeMotorRight.setSmartCurrentLimit(20);

		injestMotorLeft.setSmartCurrentLimit(20);
		injestMotorRight.setSmartCurrentLimit(20);

		intakeMotorFront.burnFlash();
		intakeMotorBack.burnFlash();
		intakeMotorLeft.burnFlash();
		intakeMotorRight.burnFlash();

		injestMotorLeft.burnFlash();
		injestMotorRight.burnFlash();
	}

	private void intake() {
		intakeMotor(intakeMotorFront);
		intakeMotor(intakeMotorBack);
		intakeMotor(intakeMotorLeft);
		intakeMotor(intakeMotorRight);
	}

	public void intakeMotor(CANSparkFlex motor) {
		motor.set(INTAKE_IN_SPEED);
    }

	private void stopIntakeMotors() {
		intakeMotorFront.set(0);
        intakeMotorBack.set(0);
        intakeMotorLeft.set(0);
        intakeMotorRight.set(0);
	}

	public void spitOut() {
		intakeMotorFront.set(INTAKE_FAST_OUT_SPEED);
        intakeMotorBack.set(INTAKE_FAST_OUT_SPEED);
        intakeMotorLeft.set(INTAKE_FAST_OUT_SPEED);
        intakeMotorRight.set(INTAKE_FAST_OUT_SPEED);
	}
}