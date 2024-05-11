package frc.team2412.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;

public class LimelightSubsystem extends SubsystemBase {

	// POTENTIAL CONSTANTS
	private GenericEntry GOAL_DISTANCE_FROM_NOTE;
	private final double AUTO_DETECTION_DISTANCE_FROM_NOTE = 3.5; // meters

	// MEMBERS

	final NetworkTable networkTable;

	String currentPoseString;

	// network tables

	// CONSTRUCTOR !
	public LimelightSubsystem() {

		// camera stream at http://10.24.12.11:5800

		// logging
		currentPoseString = "";

		networkTable = NetworkTableInstance.getDefault().getTable("limelight");
		ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");

		// helpers
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
				.addDouble("Distance From Note", this::getDistanceFromTargetInches)
				.withPosition(3, 0)
				.withSize(1, 1);
		limelightTab
				.addString("Current Pose ", this::getCurrentPoseString)
				.withPosition(0, 1)
				.withSize(4, 1);
		GOAL_DISTANCE_FROM_NOTE =
				limelightTab
						.addPersistent("Goal distance", 20.0)
						.withWidget(BuiltInWidgets.kNumberSlider)
						.withPosition(4, 0)
						.withSize(2, 1)
						.withProperties(Map.of("Min", 0.0, "Max", 30.0))
						.getEntry();
	}

	private void setPipeline() {
		networkTable.getEntry("pipeline").setNumber(3);
	}

	// METHODS

	public boolean isNoteInFront() {
		return (hasTargets() && getDistanceFromTargetInches() <= AUTO_DETECTION_DISTANCE_FROM_NOTE);
	}

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
	public double getDistanceFromTargetInches() {

		// focal length = (P x D) / W
		double focal_length = 559.0394;

		// distance = (W x F) / P
		// returns inches for testing purposes, will divide by 39.3700787 to return meters
		if (hasTargets()) {
			return (14 * focal_length) / getBoxWidth();
		} else {
			return 0.0;
		}
	}

	public String getCurrentPoseString() {
		return currentPoseString;
	}

	public boolean isWithinDistance() {
		return (getDistanceFromTargetInches() <= GOAL_DISTANCE_FROM_NOTE.getDouble(20.0));
	}

	@Override
	public void periodic() {}
}
