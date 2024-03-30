package frc.team2412.robot.util.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team2412.robot.Robot;
import frc.team2412.robot.Subsystems;
import java.util.function.Supplier;

public class AutoAlignment {
	private static Robot r = Robot.getInstance();
	private static final Subsystems s = r.subsystems;

	private static Supplier<Pose2d> currentPosition = s.drivebaseWrapper::getEstimatedPosition;
	private static Supplier<Pose2d> goalPosition = () -> AutoLogic.getSelectedAutoPath().getStartPose2d();

	private static final double POSITION_TOLERANCE = 0.5;
	private static final double ROTATION_TOLERANCE = 5;

	private static Field2d field = new Field2d();
	private static FieldObject2d robotPos;
	private static FieldObject2d robotGoalPos;
	private static ShuffleboardTab tab = Shuffleboard.getTab("AutoAlignment");

	public static void initShuffleboard() {

		tab.addString("Current Position", () -> getPose2dString(currentPosition.get()))
				.withPosition(0, 0)
				.withSize(3, 1);

		tab.addString("Auto Starting Position", () -> getPose2dString(goalPosition.get()))
				.withPosition(0, 1)
				.withSize(3, 1);

		tab.addBoolean(
						"Correct Horizontal Position", AutoAlignment::isRobotInCorrectHorizontalPosition)
				.withPosition(3, 0)
				.withSize(2, 1);
		tab.addBoolean("Correct Vertical Position", AutoAlignment::isRobotInCorrectVerticalPosition)
				.withPosition(3, 1)
				.withSize(2, 1);
		tab.addBoolean("Correct Rotation", AutoAlignment::isRobotInCorrectHorizontalPosition)
				.withPosition(3, 2)
				.withSize(2, 1);

		tab.addBoolean("Correct Position", AutoAlignment::isRobotInCorrectPosition)
				.withPosition(5, 0)
				.withSize(2, 3);

		tab.add(field).withPosition(7, 0);

		initField();
	}

	private static boolean isRobotInCorrectPosition() {
		return (isRobotInCorrectHorizontalPosition()
				&& isRobotInCorrectVerticalPosition()
				&& isRobotInCorrectRotation());
	}

	private static boolean isRobotInCorrectHorizontalPosition() {
		return MathUtil.isNear(
				currentPosition.get().getX(), goalPosition.get().getX(), POSITION_TOLERANCE);
	}

	private static boolean isRobotInCorrectVerticalPosition() {
		return MathUtil.isNear(
				currentPosition.get().getY(), goalPosition.get().getY(), POSITION_TOLERANCE);
	}

	private static boolean isRobotInCorrectRotation() {
		return MathUtil.isNear(
				currentPosition.get().getRotation().getDegrees(),
				goalPosition.get().getRotation().getDegrees(),
				ROTATION_TOLERANCE);
	}

	private static String getPose2dString(Pose2d pose) {
		return "Translation: ("
				+ pose.getX()
				+ "m, "
				+ pose.getY()
				+ "m) | Rotation: ("
				+ pose.getRotation().getDegrees()
				+ " deg.)";
	}


	private static void initField() {
		field = new Field2d();

		SmartDashboard.putData("AlignmentField", field);
		tab.add(field).withPosition(7, 0).withSize(4, 3);

		robotPos = field.getObject("Robot");
		robotGoalPos = field.getObject("AutoStartPosition");
	}

	private static void updateField() {
		robotPos.setPose(s.drivebaseWrapper.getEstimatedPosition());

		robotGoalPos.setPose(AutoLogic.getSelectedAutoPath().getStartPose2d());
	}
}
