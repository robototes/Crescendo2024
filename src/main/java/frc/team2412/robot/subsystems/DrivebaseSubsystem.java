package frc.team2412.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team2412.robot.Robot;
import frc.team2412.robot.Robot.RobotType;
import frc.team2412.robot.Subsystems.SubsystemConstants;
import java.io.File;
import java.util.EnumSet;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DrivebaseSubsystem extends SubsystemBase {

	// SWERVE CONSTANTS (that aren't in deploy dir)

	public static final double MAX_SPEED =
			Robot.getInstance().getRobotType() == RobotType.BONK
					? 3.0
					: Robot.getInstance().getRobotType() == RobotType.PRACTICE
							? 6.0
							: Robot.getInstance().getRobotType() == RobotType.CRANE ? 3.0 : 1.0;

	// Auto align stuff, dw abt it
	public static final double MAX_ACCELERATION = 3;
	public static final double MAX_ANGULAR_VELOCITY = 540;
	public static final double MAX_ANGULAR_ACCELERAITON = 720;

	// distance from center of the robot to the furthest module
	private static final double DRIVEBASE_RADIUS =
			Robot.getInstance().getRobotType() == RobotType.BONK
					? 0.305328701
					: Robot.getInstance().getRobotType() == RobotType.CRANE
							? 0.3937
							: Robot.getInstance().getRobotType() == RobotType.PRACTICE ? 0.3 : 0.3;
	private static final double JOYSTICK_DEADBAND = 0.05;
	private static final double HEADING_CORRECTION_DEADBAND = 0.005;

	// AUTO CONSTANTS

	private static final PIDConstants AUTO_TRANSLATION_PID =
			Robot.getInstance().getRobotType() == RobotType.PRACTICE
					? new PIDConstants(5, 0, 0.5) // practice
					: Robot.getInstance().getRobotType() == RobotType.BONK
							? new PIDConstants(5, 0, 0.1) // bonk
							: Robot.getInstance().getRobotType() == RobotType.CRANE
									? new PIDConstants(3.9, 0, 0.2) // crane
									: new PIDConstants(0.1, 0, 0.1); // bobot TODO: tune
	private static final PIDConstants AUTO_ROTATION_PID = new PIDConstants(5.0, 0, 0.2);
	private static final double MAX_AUTO_SPEED =
			500.0; // this seems to only affect rotation for some reason

	private final SwerveDrive swerveDrive;
	private final ShuffleboardTab drivebaseTab = Shuffleboard.getTab("Drivebase");

	private boolean xWheelsEnabled = true;
	private Rotation2d rotationSetpoint;

	// shuffleboard variables
	private GenericEntry headingCorrectionEntry;
	private GenericEntry translationSpeedEntry;
	private GenericEntry rotationSpeedEntry;
	private GenericEntry xWheelsEntry;

	public DrivebaseSubsystem() {
		initShuffleboard();

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
			case BONK:
				swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "bonkswerve");
				System.out.println("Running bonk swerve");
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

		// set drive motors to coast intially, this will be changed to brake on enable
		swerveDrive.setMotorIdleMode(false);
		// swerve drive heading will slowly drift over time as you translate. this method enables an
		// active correction using pid. disabled until testing can be done
		// TODO: this still needs to be improved
		swerveDrive.setHeadingCorrection(headingCorrectionEntry.getBoolean(true), 2.0, 0.5);
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

		// LOW verbosity only sends field position, HIGH sends full drive data, MACHINE sends data
		// viewable by AdvantageScope
		SwerveDriveTelemetry.verbosity = TelemetryVerbosity.MACHINE;
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
		if (translation.getNorm() == 0 && rotation.getRotations() == 0 && xWheelsEnabled) {
			swerveDrive.lockPose();
		}
		if (rotationSetpoint != null) {
			swerveDrive.drive(
					translation.unaryMinus(), rotationSetpoint.getRadians(), fieldOriented, false);
		} else {
			swerveDrive.drive(translation.unaryMinus(), rotation.getRadians(), fieldOriented, false);
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
											* MAX_SPEED
											* rotationSpeedEntry.getDouble(1.0)
											* -1);
					Translation2d constrainedTranslation =
							new Translation2d(
									SwerveMath.applyDeadband(forward.getAsDouble(), true, JOYSTICK_DEADBAND)
											* MAX_SPEED
											* translationSpeedEntry.getDouble(1.0),
									SwerveMath.applyDeadband(strafe.getAsDouble(), true, JOYSTICK_DEADBAND)
											* MAX_SPEED
											* translationSpeedEntry.getDouble(1.0));
					drive(constrainedTranslation, constrainedRotation, true);
				});
	}

	// this might need to be put in its own file due to complexity
	public Command rotateToAngle(Supplier<Rotation2d> angle, boolean endWhenAligned) {
		Command alignCommand =
				Commands.runEnd(
						() -> {
							rotationSetpoint =
									Rotation2d.fromRadians(
											swerveDrive
													.getSwerveController()
													.headingCalculate(
															swerveDrive.getOdometryHeading().getRadians(),
															angle.get().getRadians()));
						},
						() -> {
							rotationSetpoint = null;
						});

		if (endWhenAligned)
			return alignCommand.until(
					() ->
							Math.abs(swerveDrive.getOdometryHeading().minus(angle.get()).getRotations())
									< HEADING_CORRECTION_DEADBAND);
		return alignCommand;
	}

	public void setMotorBrake(boolean brake) {
		swerveDrive.setMotorIdleMode(brake);
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
		return SubsystemConstants.USE_APRILTAGS_CORRECTION
				? swerveDrive.getPose()
				: swerveDrive.getOdometryOnlyPose();
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

	public void toggleXWheels() {
		xWheelsEnabled = !xWheelsEnabled;
		xWheelsEntry.setBoolean(xWheelsEnabled);
	}

	public Field2d getField() {
		return swerveDrive.field;
	}

	private void initShuffleboard() {
		NetworkTableInstance inst = NetworkTableInstance.getDefault();

		headingCorrectionEntry =
				drivebaseTab
						.addPersistent("Heading Correction", true)
						.withWidget(BuiltInWidgets.kToggleSwitch)
						.withSize(2, 1)
						.getEntry();
		inst.addListener(
				headingCorrectionEntry,
				EnumSet.of(NetworkTableEvent.Kind.kValueAll),
				event -> {
					swerveDrive.setHeadingCorrection(event.valueData.value.getBoolean());
				});

		translationSpeedEntry =
				drivebaseTab
						.addPersistent("Translation Speed", 1.0)
						.withWidget(BuiltInWidgets.kNumberSlider)
						.withSize(2, 1)
						.withProperties(Map.of("Min", 0.0))
						.getEntry();
		rotationSpeedEntry =
				drivebaseTab
						.addPersistent("Rotation Speed", 1.0)
						.withWidget(BuiltInWidgets.kNumberSlider)
						.withSize(2, 1)
						.withProperties(Map.of("Min", 0.0))
						.getEntry();
		xWheelsEntry =
				drivebaseTab
						.addPersistent("X Wheels", xWheelsEnabled)
						.withWidget(BuiltInWidgets.kBooleanBox)
						.withSize(1, 1)
						.getEntry();
		xWheelsEnabled = xWheelsEntry.getBoolean(true);
	}

	/** Get the YAGSL {@link SwerveDrive} object. */
	public SwerveDrive getSwerveDrive() {
		return swerveDrive;
	}

	private SysIdRoutine getDriveSysIdRoutine() {
		return new SysIdRoutine(
				new SysIdRoutine.Config(),
				new SysIdRoutine.Mechanism(
						(Measure<Voltage> volts) -> {
							for (SwerveModule module : swerveDrive.getModules()) {
								module.getDriveMotor().setVoltage(volts.magnitude());
								module.setAngle(0);
							}
						},
						null,
						this));
	}

	private SysIdRoutine getAngleSysIdRoutine() {
		return new SysIdRoutine(
				new SysIdRoutine.Config(),
				new SysIdRoutine.Mechanism(
						(Measure<Voltage> volts) -> {
							for (SwerveModule module : swerveDrive.getModules()) {
								module.getAngleMotor().setVoltage(volts.magnitude());
							}
						},
						null,
						this));
	}

	public Command driveSysIdQuasistatic(SysIdRoutine.Direction direction) {
		return getDriveSysIdRoutine().quasistatic(direction);
	}

	public Command driveSysIdDynamic(SysIdRoutine.Direction direction) {
		return getDriveSysIdRoutine().dynamic(direction);
	}

	public Command angleSysIdQuasistatic(SysIdRoutine.Direction direction) {
		return getAngleSysIdRoutine().quasistatic(direction);
	}

	public Command angleSysIdDynamic(SysIdRoutine.Direction direction) {
		return getAngleSysIdRoutine().dynamic(direction);
	}
}
