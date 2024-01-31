package frc.team2412.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.Robot;
import frc.team2412.robot.Robot.RobotType;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

public class DrivebaseSubsystem extends SubsystemBase {

	// SWERVE CONSTANTS (that aren't in deploy dir)

	private static final double MAX_SPEED = Robot.getInstance().getRobotType() == RobotType.PRACTICE
											? 2.0
											: Robot.getInstance().getRobotType() == RobotType.CRANE
											? 3.0
											: null;
	// distance from center of the robot to the furthest module
	private static final double DRIVEBASE_RADIUS = Robot.getInstance().getRobotType() == RobotType.PRACTICE
													? 0.305328701
													: Robot.getInstance().getRobotType() == RobotType.CRANE
													? 0.3937
													: null;
	private static final double JOYSTICK_DEADBAND = 0.05;
	private static final double HEADING_CORRECTION_DEADBAND = 0.005;

	// AUTO CONSTANTS

	public static final Rotation2d MAX_ROTATIONS_PER_SEC = Rotation2d.fromRotations(0.8574);

	private static final PIDConstants AUTO_TRANSLATION_PID = new PIDConstants(0.1, 0, 0);
	private static final PIDConstants AUTO_ROTATION_PID = new PIDConstants(5.0, 0, 0);
	private static final double MAX_AUTO_SPEED =
			500.0; // this seems to only affect rotation for some reason

	private final SwerveDrive swerveDrive;

	public DrivebaseSubsystem() {
		File swerveJsonDirectory;

		switch (Robot.getInstance().getRobotType()) {
			case PRACTICE:
				swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "practiceswerve");
				System.out.println("Running practice swerve");
				break;
			case CRANE:
				swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "craneswerve");
				System.out.println("Running crane swerve");
				break;
			case COMPETITION:
			default:
				swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
				System.out.println("Running competition swerve");
		}

		try {
			swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(MAX_SPEED);
		} catch (Exception e) {
			System.out.println(e);
			throw new RuntimeException();
		}

		// set drive motors to brake
		swerveDrive.setMotorIdleMode(true);
		// enable optimization (never move the angle wheels more than 90 degrees)
		//	swerveDrive.setModuleStateOptimization(false);
		// swerve drive heading will slowly drift over time as you translate. this method enables an
		// active correction using pid. disabled until testing can be done
		swerveDrive.setHeadingCorrection(false, 0.1);
		// supposed to do something? see
		// https://broncbotz3481.github.io/YAGSL/swervelib/SwerveDrive.html#chassisVelocityCorrection
		swerveDrive.chassisVelocityCorrection = true;

		swerveDrive.synchronizeModuleEncoders();

		// Configure auto builder for PathPlanner
		AutoBuilder.configureHolonomic(
				this::getPose,
				this::setPose,
				this::getRobotSpeeds,
				this::drive,
				new HolonomicPathFollowerConfig(
						AUTO_TRANSLATION_PID,
						AUTO_ROTATION_PID,
						MAX_AUTO_SPEED,
						DRIVEBASE_RADIUS,
						new ReplanningConfig()),
				() ->
						DriverStation.getAlliance()
								.get()
								.equals(Alliance.Red), // flip path if on the red alliance
				this);
	}

	/**
	 * Drive with robot-relative chassis speeds
	 *
	 * @param speeds Robot-relative speeds
	 */
	public void drive(ChassisSpeeds speeds) {
		swerveDrive.drive(speeds);
	}

	/**
	 * Drive the robot
	 *
	 * @param fieldOriented Whether these values are field oriented
	 */
	public void drive(Translation2d translation, Rotation2d rotation, boolean fieldOriented) {
		// if we're requesting the robot to stay still, lock wheels in X formation
		if (translation.getNorm() == 0 && rotation.getRotations() == 0) {
			swerveDrive.lockPose();
		} else {
			swerveDrive.drive(translation.unaryMinus(), -rotation.getRadians(), fieldOriented, false);
		}
	}

	/**
	 * Drives the robot using joystick inputs
	 *
	 * @param forward Forward motion in meters. A negative value makes the robot go backwards
	 * @param strafe Strafe motion in meters. A negative value makes the robot go left
	 * @param rotation Rotation2d value of robot rotation. CW is positive TODO: is this true?
	 */
	public Command driveJoystick(
			DoubleSupplier forward, DoubleSupplier strafe, Supplier<Rotation2d> rotation) {
		return this.run(
				() -> {
					Rotation2d constrainedRotation =
							Rotation2d.fromRotations(
									SwerveMath.applyDeadband(rotation.get().getRotations(), true, JOYSTICK_DEADBAND)
											* MAX_SPEED);
					Translation2d constrainedTranslation =
							new Translation2d(
									SwerveMath.applyDeadband(forward.getAsDouble(), true, JOYSTICK_DEADBAND)
											* MAX_SPEED,
									SwerveMath.applyDeadband(strafe.getAsDouble(), true, JOYSTICK_DEADBAND)
											* MAX_SPEED);
					drive(constrainedTranslation, constrainedRotation, true);
				});
	}

	public ChassisSpeeds getRobotSpeeds() {
		return swerveDrive.getRobotVelocity();
	}

	/** Set the robot's pose. TODO: does this change yaw too? does this affect field oriented? */
	public void setPose(Pose2d pose) {
		swerveDrive.resetOdometry(pose);
	}

	/** Get the robot's pose */
	public Pose2d getPose() {
		return swerveDrive.getPose();
	}

	/**
	 * Reset the gyro angle. After this method is called, yaw will be zero. Pose is also updated to
	 * zero rotation but same position
	 */
	public void resetGyro() {
		swerveDrive.zeroGyro();
	}

	/** Reset everything we can on the drivebase. To be used before auto starts */
	public void resetRobot() {
		swerveDrive.resetDriveEncoders();
		resetGyro();
		setPose(new Pose2d());
	}
}
