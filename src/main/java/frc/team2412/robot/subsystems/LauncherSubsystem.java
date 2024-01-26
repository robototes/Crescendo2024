package frc.team2412.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.Hardware;

public class LauncherSubsystem extends SubsystemBase {
	// CONSTANTS
	// MOTOR SPEED VALUES
	public static final double SPEAKER_SHOOT_SPEED = 0.5;
	public static final double AMP_SHOOT_SPEED = 0.2;
	public static final double ANGLE_CHANGE_SPEED = 0.15;

	// HARDWARE
	private final CANSparkFlex launcherTopMotor;
	private final CANSparkFlex launcherBottomMotor;
	private final CANSparkFlex launcherAngleMotor;
	private final CANSparkFlex launcherHoodMotor;
	private final SparkAbsoluteEncoder launcherAngleEncoder;
	private final SparkAbsoluteEncoder launcherHoodEncoder;
	private final SparkPIDController launcherAnglePIDController;
	private final SparkPIDController launcherHoodPIDController;

	GenericEntry launcherSpeed =
			Shuffleboard.getTab("Launcher")
					.addPersistent("Launcher Speed", SPEAKER_SHOOT_SPEED)
					.withSize(2, 1)
					.withWidget(BuiltInWidgets.kTextView)
					.getEntry();
	// Constructor
	public LauncherSubsystem() {

		// MOTOR INSTANCE VARIBLES
		launcherTopMotor = new CANSparkFlex(Hardware.LAUNCHER_TOP_MOTOR_ID, MotorType.kBrushless);
		launcherBottomMotor = new CANSparkFlex(Hardware.LAUNCHER_BOTTOM_MOTOR_ID, MotorType.kBrushless);
		launcherAngleMotor = new CANSparkFlex(Hardware.LAUNCHER_ANGLE_MOTOR_ID, MotorType.kBrushless);
		launcherHoodMotor = new CANSparkFlex(Hardware.LAUNCHER_HOOD_MOTOR_ID, MotorType.kBrushless);
		launcherAngleEncoder =
				launcherAngleMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
		launcherHoodEncoder =
				launcherHoodMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
		launcherAnglePIDController = launcherAngleMotor.getPIDController();
		launcherHoodPIDController = launcherHoodMotor.getPIDController();

		launcherAnglePIDController.setFeedbackDevice(launcherAngleEncoder);
		launcherHoodPIDController.setFeedbackDevice(launcherHoodEncoder);
	}

	// stop specific motor method
	public void stopMotor(CANSparkFlex motor) {
		motor.stopMotor();
	}

	public void shoot(double speed) {
		launcherTopMotor.set(speed);
		launcherBottomMotor.set(-speed);
	}
	// returns the degrees of the angle of the launcher
	public double getAngle() {
		// get position returns a double in the form of rotations
		return Units.rotationsToDegrees(launcherAngleEncoder.getPosition());
	}

	public void setAngle(double angle) {
		launcherAnglePIDController.setReference(Units.degreesToRotations(angle), ControlType.kPosition);
	}

	public double getHoodAngle() {
		return Units.rotationsToDegrees(launcherHoodEncoder.getPosition());
	}

	public void setHoodAngle(double angle) {
		launcherHoodPIDController.setReference(Units.degreesToRotations(angle), ControlType.kPosition);
	}

	public void DistanceModifier(double distance, double modifier) {
		double speed = distance / modifier;
	}
}
