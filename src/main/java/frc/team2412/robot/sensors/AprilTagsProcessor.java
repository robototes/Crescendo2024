package frc.team2412.robot.sensors;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import frc.team2412.robot.Hardware;
import frc.team2412.robot.util.DrivebaseWrapper;
import frc.team2412.robot.util.SendablePose3d;
import java.util.EnumSet;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * All 3D poses and transforms use the NWU (North-West-Up) coordinate system, where +X is
 * north/forward, +Y is west/left, and +Z is up. On the field, this is based on the blue driver
 * station. (+X is forward from blue driver station, +Y is left, and +Z is up. Additionally, the
 * origin is the right corner of the blue driver station- All coordinates should be non-negative.)
 *
 * <p>Rotations follow the right-hand rule- If the thumb of a right hand is pointed in a positive
 * axis, the fingers curl in the direction of positive rotation. Alternatively, the rotation is CCW+
 * looking into the positive axis.
 *
 * <p>2D field poses are just the projection of the 3D pose onto the XY plane.
 */
public class AprilTagsProcessor {
	public static final Transform3d ROBOT_TO_CAM =
			new Transform3d(
					Units.inchesToMeters(27.0 / 2.0 - 0.94996),
					0,
					Units.inchesToMeters(8.12331),
					new Rotation3d(0, Units.degreesToRadians(-30), 0));

	// TODO Measure these
	private static final Vector<N3> STANDARD_DEVS = VecBuilder.fill(1, 1, Units.degreesToRadians(30));

	private static PhotonPipelineResult copy(PhotonPipelineResult result) {
		var copy =
				new PhotonPipelineResult(
						result.getLatencyMillis(), result.targets, result.getMultiTagResult());
		copy.setTimestampSeconds(result.getTimestampSeconds());
		return copy;
	}

	private static final double MAX_POSE_AMBIGUITY = 0.1;

	// Both of these are in radians
	// Negate to convert robot to cam to cam to robot, and negate again because tag is flipped,
	// leading to no net effect
	private static final double EXPECTED_CAM_TO_TARGET_PITCH = ROBOT_TO_CAM.getRotation().getY();
	private static final double CAM_TO_TARGET_PITCH_TOLERANCE = 0.1;

	static {
		System.out.println("Expected pitch: " + EXPECTED_CAM_TO_TARGET_PITCH);
	}

	private static boolean hasCorrectPitch(Transform3d camToTarget) {
		return Math.abs(camToTarget.getRotation().getY() - EXPECTED_CAM_TO_TARGET_PITCH)
				< CAM_TO_TARGET_PITCH_TOLERANCE;
	}

	private static PhotonTrackedTarget swapBestAndAltTransforms(PhotonTrackedTarget target) {
		return new PhotonTrackedTarget(
				target.getYaw(),
				target.getPitch(),
				target.getArea(),
				target.getSkew(),
				target.getFiducialId(),
				target.getAlternateCameraToTarget(), // Swap
				target.getBestCameraToTarget(),
				target.getPoseAmbiguity(),
				target.getMinAreaRectCorners(),
				target.getDetectedCorners());
	}

	private static PhotonPipelineResult filteredPipelineResult(PhotonPipelineResult result) {
		var copy = copy(result);
		for (int i = copy.targets.size() - 1; i >= 0; --i) {
			var target = copy.targets.get(i);
			if (!hasCorrectPitch(target.getBestCameraToTarget())) {
				if (hasCorrectPitch(target.getAlternateCameraToTarget())) {
					target = swapBestAndAltTransforms(target);
					copy.targets.set(i, target);
					if (canPrint) {
						System.out.println(
								"Swapped best and alt transform for target "
										+ i
										+ " (best pitch="
										+ target.getBestCameraToTarget().getRotation().getY()
										+ ", alt pitch="
										+ target.getAlternateCameraToTarget().getRotation().getY()
										+ ")");
					}
				} else {
					copy.targets.remove(i);
					if (canPrint) {
						System.out.println(
								"Removed target "
										+ i
										+ " (best pitch="
										+ target.getBestCameraToTarget().getRotation().getY()
										+ ", alt pitch="
										+ target.getAlternateCameraToTarget().getRotation().getY()
										+ ") because of bad pitch ");
					}
					continue;
				}
				if (target.getPoseAmbiguity() > MAX_POSE_AMBIGUITY) {
					copy.targets.remove(i);
					if (canPrint) {
						System.out.println(
								"Excluded target " + i + " with ambiguity " + target.getPoseAmbiguity());
					}
					continue;
				}
			}
		}
		return copy;
	}

	private final PhotonCamera photonCamera;
	private final PhotonPoseEstimator photonPoseEstimator;
	private final DrivebaseWrapper aprilTagsHelper;
	private final FieldObject2d rawVisionFieldObject;

	// These are always set with every pipeline result
	private PhotonPipelineResult latestResult = null;
	private PhotonPipelineResult latestFilteredResult = null;
	private Optional<EstimatedRobotPose> latestPose = Optional.empty();

	// These are only set when there's a valid pose
	private double lastTimestampSeconds = 0;
	private Pose2d lastFieldPose = new Pose2d(-1, -1, new Rotation2d());

	private static double lastPrintTimeSeconds = -1;
	private static boolean canPrint = false;

	private static final AprilTagFieldLayout fieldLayout =
			AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

	public AprilTagsProcessor(DrivebaseWrapper aprilTagsHelper) {
		this.aprilTagsHelper = aprilTagsHelper;
		rawVisionFieldObject = aprilTagsHelper.getField().getObject("RawVision");
		var networkTables = NetworkTableInstance.getDefault();
		// if (Robot.isSimulation()) {
		// 	networkTables.stopServer();
		// 	networkTables.setServer(Hardware.PHOTON_IP);
		// 	networkTables.startClient4("Photonvision");
		// }

		photonCamera = new PhotonCamera(Hardware.PHOTON_CAM);
		photonPoseEstimator =
				new PhotonPoseEstimator(
						fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_RIO, photonCamera, ROBOT_TO_CAM);

		photonPoseEstimator.setLastPose(aprilTagsHelper.getEstimatedPosition());
		photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_LAST_POSE);

		networkTables.addListener(
				networkTables
						.getTable("photonvision")
						.getSubTable(Hardware.PHOTON_CAM)
						.getEntry("rawBytes"),
				EnumSet.of(NetworkTableEvent.Kind.kValueAll),
				event -> update());

		ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("AprilTags");
		shuffleboardTab.addBoolean("Has targets", this::hasTargets).withPosition(0, 0).withSize(1, 1);
		shuffleboardTab
				.addInteger("Num targets", this::getNumTargets)
				.withPosition(0, 1)
				.withSize(1, 1);
		shuffleboardTab
				.addDouble("Last timestamp", this::getLastTimestampSeconds)
				.withPosition(1, 0)
				.withSize(1, 1);
		shuffleboardTab
				.add("3d pose on field", new SendablePose3d(this::getRobotPose))
				.withPosition(2, 0)
				.withSize(1, 6);
	}

	public void update() {
		double now = Timer.getFPGATimestamp();
		canPrint = now - lastPrintTimeSeconds >= 1;
		if (canPrint) {
			lastPrintTimeSeconds = now;
			System.out.println("Got update at " + now);
		}
		latestResult = photonCamera.getLatestResult();
		latestFilteredResult = filteredPipelineResult(latestResult);
		latestPose = photonPoseEstimator.update(latestFilteredResult);
		if (latestPose.isPresent()) {
			if (canPrint) {
				System.out.println("Have result");
			}
			lastTimestampSeconds = latestPose.get().timestampSeconds;
			lastFieldPose = latestPose.get().estimatedPose.toPose2d();
			rawVisionFieldObject.setPose(lastFieldPose);
			aprilTagsHelper.addVisionMeasurement(lastFieldPose, lastTimestampSeconds, STANDARD_DEVS);
			var estimatedPose = aprilTagsHelper.getEstimatedPosition();
			aprilTagsHelper.getField().setRobotPose(estimatedPose);
			photonPoseEstimator.setLastPose(estimatedPose);
		}
	}

	public boolean hasTargets() {
		return latestPose.isPresent();
	}

	public int getNumTargets() {
		return latestResult == null ? -1 : latestResult.getTargets().size();
	}

	/**
	 * Calculates the robot pose using the best target. Returns null if there is no known robot pose.
	 *
	 * @return The calculated robot pose in meters.
	 */
	public Pose3d getRobotPose() {
		if (latestPose.isPresent()) {
			return latestPose.get().estimatedPose;
		}
		return null;
	}

	/**
	 * Returns the last time we saw an AprilTag.
	 *
	 * @return The time we last saw an AprilTag in seconds since FPGA startup.
	 */
	public double getLastTimestampSeconds() {
		return lastTimestampSeconds;
	}
}
