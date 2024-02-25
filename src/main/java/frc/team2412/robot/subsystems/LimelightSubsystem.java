package frc.team2412.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

	// meters?
	public static final double CAMERA_MOUNT_HEIGHT = 0.1143;
	public static final double CAMERA_ANGLE_OFFSET = 0;
	public static final double TARGET_HEIGHT = 0.33;

	public static final double GOAL_DISTANCE_FROM_TARGET = 0.7;
	public static final double GOAL_DISTANCE_FROM_CONE = 0.3;
	public static final double GOAL_DISTANCE_FROM_CUBE = 0.3;

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
				.addDouble("Target Distance 2 - TEST ", this::getDistanceFromTarget)
				.withPosition(3, 0)
				.withSize(1, 1);
		limelightTab
				.addDouble("Limelight Based Turn Power - TEST ", this::turnPowerLin)
				.withPosition(4, 0)
				.withSize(1, 1);
		limelightTab
				.addDouble("Limelight Based Drive Power - TEST ", this::drivePowerLin)
				.withPosition(5, 0)
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

	// returns turn power from exponential curve, input is from -27deg to +27deg
	public double turnPowerExp() {
		if (Math.abs(getHorizontalOffset()) >= 1.0) {
			return MathUtil.clamp(
					(Math.signum(getHorizontalOffset())
							* (Math.pow(Math.abs(getHorizontalOffset()), 0.6) * 0.14)),
					-0.25,
					0.25);
		} else {
			return 0.0;
		}
	}

	// returns turn power from linear curve, input is from -27deg to +27deg
	public double turnPowerLin() {
		if (Math.abs(getHorizontalOffset()) >= 1.0) {
			return MathUtil.clamp(0.01 * getHorizontalOffset(), -0.25, 0.25);
		} else {
			return 0.0;
		}
	}

	// drive power returns linear
	public double drivePowerLin() {
		if (Math.abs(getDistanceFromTarget()) >= 20) {
			return MathUtil.clamp(0.01 * getDistanceFromTarget() - 0.2, 0, 0.25);
		} else {
			return 0.0;
		}
	}

	// tan(degree) * distance = sideways distance

	// target height / tan(vertical angle)

	public Pose2d getTargetPose(Pose2d currentPose) {

		// math thing to get target pose using current pose

		Rotation2d targetHeading =
				new Rotation2d(
						currentPose.getRotation().getRadians() + Units.degreesToRadians(getHorizontalOffset()));
		double targetDistance = getDistanceFromTarget() / 39.3700787;

		double targetX = Math.sin(targetHeading.getRadians()) * targetDistance;
		double targetY = Math.cos(targetHeading.getRadians()) * targetDistance;

		Pose2d targetPose =
				new Pose2d(currentPose.getX() + targetY, currentPose.getY() + targetX, targetHeading);

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
		return (getDistanceFromTarget() <= GOAL_DISTANCE_FROM_TARGET);
	}

	@Override
	public void periodic() {}
}
