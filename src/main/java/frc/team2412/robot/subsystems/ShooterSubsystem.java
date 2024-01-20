package frc.team2412.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.Hardware;

public class ShooterSubsystem extends SubsystemBase {
	// CONSTANTS
	// MOTOR SPEED VALUES
	public static final double SPEAKER_SHOOT_SPEED = 0.8;
	public static final double AMP_SHOOT_SPEED = 0.2;
	public static final double ANGLE_CHANGE_SPEED = 0.15;

	// HARDWARE
	private final TalonFX shooterTopMotor;
	private final TalonFX shooterBottomMotor;
	private final CANSparkMax shooterAngleMotor;
	private final SparkAbsoluteEncoder shooterAngleEncoder;
	private final SparkPIDController shooterAnglePidController;
	// Constructor
	public ShooterSubsystem() {

		// MOTOR INSTANCE VARIBLES
		shooterTopMotor = new TalonFX(Hardware.SHOOTER_TOP_MOTOR_ID);
		shooterBottomMotor = new TalonFX(Hardware.SHOOTER_BOTTOM_MOTOR_ID);
		shooterAngleMotor = new CANSparkMax(Hardware.SHOOTER_ANGLE_MOTOR_ID, MotorType.kBrushless);
		shooterAngleEncoder =
				shooterAngleMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
		shooterAnglePidController = shooterAngleMotor.getPIDController();
	}

	// drive specific motor method
	public void driveMotor(TalonFX motor, double speed) {
		motor.set(speed);
	}

	// stop specific motor method
	public void stopMotor(TalonFX motor) {
		motor.stopMotor();
	}

	public void shoot(double speed) {
		shooterTopMotor.set(speed);
		shooterBottomMotor.set(speed);
	}

	public double getAngle() {
		return Units.rotationsToDegrees(shooterAngleEncoder.getPosition());
	}

	public void setAngle(double angle) {
		shooterAnglePidController.setReference(Units.degreesToRotations(angle), ControlType.kPosition);
	}
}
