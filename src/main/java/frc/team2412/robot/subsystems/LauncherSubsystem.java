package frc.team2412.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.Hardware;
import frc.team2412.robot.util.SparkPIDWidget;

import java.util.Map;

public class LauncherSubsystem extends SubsystemBase {
	// CONSTANTS
	// MOTOR SPEED VALUES
	// max Free Speed: 6784 RPM
	public static final double SPEAKER_SHOOT_SPEED = 0.5;
	public static final double AMP_SHOOT_SPEED = 0.2;
	public static final double ANGLE_CHANGE_SPEED = 0.15;
	public static final double ANGLE_TOLERANCE = 0.5;

	// RPM equivalents of the above values
	public static final int SPEAKER_SHOOT_SPEED_RPM = 3392; // 50%
	public static final int AMP_SHOOT_SPEED_RPM = 1356; // 20%
	public static final double ANGLE_CHANGE_SPEED_RPM = 1017; // 15%

	// HARDWARE
	private final CANSparkFlex launcherTopMotor;
	private final CANSparkFlex launcherBottomMotor;
	private final CANSparkFlex launcherAngleMotor;
	private final RelativeEncoder launcherTopEncoder;
	private final RelativeEncoder launcherBottomEncoder;
	private final SparkAbsoluteEncoder launcherAngleEncoder;
	private final SparkPIDController launcherAnglePIDController;
	private final SparkPIDController launcherTopPIDController;
	private final SparkPIDController launcherBottomPIDController;

	private final GenericEntry setLauncherSpeedEntry =
			Shuffleboard.getTab("Launcher")
					.addPersistent("Launcher Speed setpoint", SPEAKER_SHOOT_SPEED_RPM)
					.withSize(3, 1)
					.withWidget(BuiltInWidgets.kNumberSlider)
					.withProperties(Map.of("Min", -6784, "Max", 6784))
					.getEntry();

	private final GenericEntry launcherAngleEntry =
			Shuffleboard.getTab("Launcher")
					.add("Launcher angle", 0)
					.withSize(1, 1)
					.withWidget(BuiltInWidgets.kTextView)
					.getEntry();
	private final GenericEntry launcherSpeedEntry =
			Shuffleboard.getTab("Launcher")
					.add("Launcher Speed", 0)
					.withSize(1, 1)
					.withWidget(BuiltInWidgets.kTextView)
					.getEntry();
	// Constructor
	public LauncherSubsystem() {

		// MOTOR INSTANCE VARIBLES
		// motors
		launcherTopMotor = new CANSparkFlex(Hardware.LAUNCHER_TOP_MOTOR_ID, MotorType.kBrushless);
		launcherBottomMotor = new CANSparkFlex(Hardware.LAUNCHER_BOTTOM_MOTOR_ID, MotorType.kBrushless);
		launcherAngleMotor = new CANSparkFlex(Hardware.LAUNCHER_ANGLE_MOTOR_ID, MotorType.kBrushless);
		// encoders
		launcherTopEncoder = launcherTopMotor.getEncoder();
		launcherBottomEncoder = launcherBottomMotor.getEncoder();
		launcherAngleEncoder = launcherAngleMotor.getAbsoluteEncoder(Type.kDutyCycle);

		// PID controllers
		// Create launcherTopPIDController and launcherTopMotor]
		launcherTopPIDController = launcherTopMotor.getPIDController();
		launcherTopPIDController.setFeedbackDevice(
				launcherTopEncoder);
		launcherBottomPIDController = launcherBottomMotor.getPIDController();
		launcherBottomPIDController.setFeedbackDevice(
				launcherBottomEncoder);
		launcherAnglePIDController = launcherAngleMotor.getPIDController();
		launcherAnglePIDController.setFeedbackDevice(
				launcherAngleEncoder);
		Shuffleboard.getTab("Launcher").add(new SparkPIDWidget(launcherAnglePIDController,  "launcherAnglePIDController"));
		Shuffleboard.getTab("Launcher").add(new SparkPIDWidget(launcherTopPIDController, "launcherTopPIDController"));
		Shuffleboard.getTab("Launcher").add(new SparkPIDWidget(launcherBottomPIDController, "launcherBottomPIDController"));
	}

	public void configMotors() {
		launcherTopMotor.restoreFactoryDefaults();
		launcherBottomMotor.restoreFactoryDefaults();
		launcherAngleMotor.restoreFactoryDefaults();
		// idle mode (wow)
		launcherTopMotor.setIdleMode(
				IdleMode.kCoast);
		launcherBottomMotor.setIdleMode(
				IdleMode.kCoast);
		launcherAngleMotor.setIdleMode(
				IdleMode.kBrake);
		// inveritng the bottom motor lmao
		launcherBottomMotor.setInverted(true);

		// current limit
		launcherTopMotor.setSmartCurrentLimit(20);
		launcherBottomMotor.setSmartCurrentLimit(
				20);
		launcherAngleMotor.setSmartCurrentLimit(
				20);

		launcherTopMotor.burnFlash();
		launcherBottomMotor.burnFlash();
		launcherAngleMotor.burnFlash();
	}

	// stop launcher motors method
	public void stopLauncher() {
		launcherTopMotor.stopMotor();
		launcherBottomMotor.stopMotor();
	}
	// uses the value from the entry
	public void launch() {
		double speed = setLauncherSpeedEntry.getDouble(SPEAKER_SHOOT_SPEED);
		launcherTopPIDController.setReference(speed, ControlType.kVelocity);
		launcherBottomPIDController.setReference(speed, ControlType.kVelocity);
	}
	// used for presets
	public void launch(double speed) {
		launcherTopPIDController.setReference(speed, ControlType.kVelocity);
		launcherBottomPIDController.setReference(speed, ControlType.kVelocity);
	}
	// returns the degrees of the angle of the launcher
	public double getAngle() {
		// get position returns a double in the form of rotations per minute
		return Units.rotationsToDegrees(launcherAngleEncoder.getPosition());
	}

	public void setAngle(double angle) {
		launcherAnglePIDController.setReference(
				Units.degreesToRotations(angle),
				ControlType.kPosition);
	}

	@Override
	public void periodic() {
		launcherAngleEntry.setDouble(getAngle());
		launcherSpeedEntry.setDouble(
				launcherTopEncoder.getVelocity());
	}
}
