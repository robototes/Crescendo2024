package frc.team2412.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

	// POTENTIAL CONSTANTS
	public static final double GOAL_DISTANCE_FROM_NOTE = 20.0;

	// MEMBERS

	final NetworkTable networkTable;

	String currentPoseString;
	String targetPoseString;

	// network tables

	// CONSTRUCTOR !
	public LimelightSubsystem() {

		// camera stream at http://10.24.12.21:5800

		// logging
		currentPoseString = "";
		targetPoseString = "";

		networkTable = NetworkTableInstance.getDefault().getTable("limelight");
		ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");

		setPipeline();

		limelightTab.addBoolean("hasTarget", this::hasTargets).withPosition(0, 0).withSize(1, 1);
		limelightTab
				.addDouble("Horizontal Offset", this::getHorizontalOffset)
				.withPosition(1, 0)
				.withSize(1, 1);
		limelightTab
				.addDouble("Vertical Offset", this::getVerticalOffset)
				.withPosition(2, 0)
				.withSize(1, 1);
		limelightTab
				.addDouble("Distance From Note", this::getDistanceFromTarget)
				.withPosition(3, 0)
				.withSize(1, 1);
		limelightTab
				.addString("Current Pose ", this::getCurrentPoseString)
				.withPosition(0, 1)
				.withSize(4, 1);
		limelightTab
				.addString("Target Pose ", this::getTargetPoseString)
				.withPosition(0, 2)
				.withSize(4, 1);
	}

	private void setPipeline() {
		networkTable.getEntry("pipeline").setNumber(0);
	}

	// METHODS

	public boolean hasTargets() {
		return (networkTable.getEntry("tv").getDouble(0) != 0);
	}

	public double getHorizontalOffset() {
		return networkTable.getEntry("tx").getDouble(0);
	}

	public double getVerticalOffset() {
		return networkTable.getEntry("ty").getDouble(0);
	}

	public double getBoxWidth() {
		return networkTable.getEntry("tlong").getDouble(0);
	}
	// TODO re-measure distances to make this right
	public double getDistanceFromTarget() {

		// focal length = (P x D) / W
		double focal_length = 559.0394;

		// distance = (W x F) / P
		// returns inches for testing purposes, will divide by 39.3700787 to return meters
		return (14 * focal_length) / getBoxWidth();
	}

	// tan(degree) * distance = sideways distance

	// target height / tan(vertical angle)

	public Pose2d getTargetPose(Pose2d currentPose) {

		// math thing to get target pose using current pose

		Rotation2d targetHeading =
				new Rotation2d(
						currentPose.getRotation().getRadians() + Units.degreesToRadians(getHorizontalOffset()));
		double targetDistance = getDistanceFromTarget() / 39.3700787;

		Translation2d translationOffset = new Translation2d(targetDistance, targetHeading);
		Pose2d targetPose =
				new Pose2d(currentPose.getTranslation().plus(translationOffset), targetHeading);

		currentPoseString = currentPose.toString();
		targetPoseString = targetPose.toString();

		return targetPose;
	}

	public String getCurrentPoseString() {
		return currentPoseString;
	}

	public String getTargetPoseString() {
		return targetPoseString;
	}

	public boolean isWithinDistance() {
		return (getDistanceFromTarget() <= GOAL_DISTANCE_FROM_NOTE);
	}

	@Override
	public void periodic() {}
}
