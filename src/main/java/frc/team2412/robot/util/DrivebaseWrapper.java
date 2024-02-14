package frc.team2412.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.SwerveDrive;

/**
 * Wrapper class around members of {@link SwerveDrive}, like the Field2d telemetry and the pose
 * estimator.
 */
public class DrivebaseWrapper {
	@FunctionalInterface
	private static interface VisionMeasurementAdder {
		void addVisionMeasurement(Pose2d robotPose, double timestamp, Matrix<N3, N1> stdDevs);
	}

	private final Field2d field;

	// Use the pose estimator ONLY for thread-safe operations!
	private final SwerveDrivePoseEstimator poseEstimator;

	// Since adding a vision measurement is not thread-safe, use a functional interface that can use
	// SwerveDrive's methods (which are thread-safe)
	private final VisionMeasurementAdder visionMeasurementAdder;

	/** Constructor for when there is no drivebase. */
	public DrivebaseWrapper() {
		field = new Field2d();
		// YAGSL will normally do this for us
		// This may be pointless though since Field2d can't be shown in both SmartDashboard and the
		// MatchDashboard tab
		SmartDashboard.putData("Field", field);
		// Because there's no drivebase attached, we aren't using odometry and so module locations etc.
		// don't matter (but kinematics requires at least 2 module positions)
		var dummyModulePositions =
				new SwerveModulePosition[] {new SwerveModulePosition(), new SwerveModulePosition()};
		poseEstimator =
				new SwerveDrivePoseEstimator(
						new SwerveDriveKinematics(new Translation2d(1, 1), new Translation2d(-1, -1)),
						new Rotation2d(),
						dummyModulePositions,
						new Pose2d());
		visionMeasurementAdder = poseEstimator::addVisionMeasurement;
		// The pose estimator will silently ignore updates if the internal pose buffer is empty, so add
		// an odometry measurement to get started
		poseEstimator.updateWithTime(0, new Rotation2d(), dummyModulePositions);
	}

	/**
	 * Constructor for when there is a drivebase.
	 *
	 * @param swerveDrive The drivebase object to use.
	 */
	public DrivebaseWrapper(SwerveDrive swerveDrive) {
		field = swerveDrive.field;
		poseEstimator = swerveDrive.swerveDrivePoseEstimator;
		visionMeasurementAdder = swerveDrive::addVisionMeasurement;
	}

	/**
	 * The {@link Field2d} object to use for telemetry.
	 *
	 * @return The Field2d object.
	 */
	public Field2d getField() {
		return field;
	}

	/**
	 * Adds a vision measurement with the specified standard deviations.
	 *
	 * @param robotPose Pose of the robot on the field in meters.
	 * @param timestamp Timestamp of the vision measurement in seconds since FPGA startup.
	 * @param stdDevs The standard deviations of the vision measurement. (X position in meters, Y
	 *     position in meters, and heading in radians)
	 * @see SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)
	 */
	public void addVisionMeasurement(Pose2d robotPose, double timestamp, Matrix<N3, N1> stdDevs) {
		visionMeasurementAdder.addVisionMeasurement(robotPose, timestamp, stdDevs);
	}

	/**
	 * Return the pose estimator's estimate of the robot position.
	 *
	 * @return The estimated pose.
	 */
	public Pose2d getEstimatedPosition() {
		return poseEstimator.getEstimatedPosition();
	}
}
