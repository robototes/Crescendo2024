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
import java.util.Map;

public class LauncherSubsystem extends SubsystemBase {
	// CONSTANTS
	// MOTOR SPEED VALUES
	public static final double SPEAKER_SHOOT_SPEED = 0.5;
	public static final double AMP_SHOOT_SPEED = 0.2;
	public static final double ANGLE_CHANGE_SPEED = 0.15;
	public static final double ANGLE_TOLERANCE = 0.5;

	// HARDWARE
	private final CANSparkFlex launcherTopMotor;
	private final CANSparkFlex launcherBottomMotor;
	private final CANSparkFlex launcherAngleMotor;
	private final RelativeEncoder launcherTopEncoder;
	private final RelativeEncoder launcherBottomEncoder;
	private final SparkAbsoluteEncoder launcherAngleEncoder;
	private final SparkPIDController launcherAnglePIDController;

	private final GenericEntry setLauncherSpeedEntry =
			Shuffleboard.getTab("Launcher")
					.addPersistent("Launcher Speed setpoint", SPEAKER_SHOOT_SPEED)
					.withSize(2, 1)
					.withWidget(BuiltInWidgets.kNumberSlider)
					.withProperties(Map.of("Min", -1, "Max", 1))
					.getEntry();

	private final GenericEntry launcherAngleEntry =
			Shuffleboard.getTab("Launcher")
					.add("Launcher angle", getAngle())
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
		launcherAnglePIDController = launcherAngleMotor.getPIDController();
		launcherAnglePIDController.setFeedbackDevice(launcherAngleEncoder);
	}

	public void configMotors() {
		launcherTopMotor.restoreFactoryDefaults();
		launcherBottomMotor.restoreFactoryDefaults();
		launcherAngleMotor.restoreFactoryDefaults();
		// idle mode (wow)
		launcherTopMotor.setIdleMode(IdleMode.kCoast);
		launcherBottomMotor.setIdleMode(IdleMode.kCoast);
		launcherAngleMotor.setIdleMode(IdleMode.kBrake);
		// inveritng the bottom motor lmao
		launcherBottomMotor.setInverted(true);

		// current limit
		launcherTopMotor.setSmartCurrentLimit(20);
		launcherBottomMotor.setSmartCurrentLimit(20);
		launcherAngleMotor.setSmartCurrentLimit(20);

		launcherTopMotor.burnFlash();
		launcherBottomMotor.burnFlash();
		launcherAngleMotor.burnFlash();
	}

	// stop launcher motors method
	public void stopLauncher() {
		launcherTopMotor.stopMotor();
		launcherBottomMotor.stopMotor();
	}

	public void launch() {
		double speed = setLauncherSpeedEntry.getDouble(SPEAKER_SHOOT_SPEED);
		launcherTopMotor.set(speed);
		launcherBottomMotor.set(speed);
	}

	public void launch(double speed){
		launcherTopMotor.set(speed);
		launcherBottomMotor.set(speed);
	}
	// returns the degrees of the angle of the launcher
	public double getAngle() {
		// get position returns a double in the form of rotations
		return Units.rotationsToDegrees(launcherAngleEncoder.getPosition());
	}

	public void setAngle(double angle) {
		launcherAnglePIDController.setReference(Units.degreesToRotations(angle), ControlType.kPosition);
	}

	@Override
	public void periodic() {
		// .get will be replaced with .getVelocity once PID is established for flywheels :C
		launcherAngleEntry.setDouble(getAngle());
		launcherSpeedEntry.setDouble(launcherTopMotor.get());
	}
}
