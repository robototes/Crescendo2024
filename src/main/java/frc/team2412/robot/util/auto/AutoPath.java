package frc.team2412.robot.util.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.util.auto.AutoLogic.StartPosition;

public class AutoPath {

	private final Pose2d startPose2d;
	private final StartPosition startPosition;
	private final String displayName;
	private final Command autoCommand;
	private final boolean vision;

	public AutoPath(String displayName, String pathPlannerAutoName, boolean vision) {
		this.displayName = displayName;
		startPose2d = PathPlannerAuto.getStaringPoseFromAutoFile(pathPlannerAutoName);
		autoCommand = AutoBuilder.buildAuto(pathPlannerAutoName);
		this.vision = vision;

		// in the case that the auto for whatever reason's starting pose is slightly off,
		// is still able to match with a startPosition if it is close enough
		for (StartPosition pos : StartPosition.values()) {
			if (matchesStartPosition(pos)) {
				startPosition = pos;
				return;
			}
		}
		startPosition = StartPosition.MISC;
	}

	public AutoPath(String displayName, String pathPlannerAutoName) {
		this(displayName, pathPlannerAutoName, false);
	}

	public StartPosition getStartPose() {
		return startPosition;
	}

	public Pose2d getStartPose2d() {
		return startPose2d;
	}

	public String getDisplayName() {
		return displayName;
	}

	public Command getAutoCommand() {
		return autoCommand;
	}

	public boolean isVision() {
		return vision;
	}

	/**
	 * Checks the x, y, and rotation of the auto's starting position and compares it with the expected
	 * starting position, returning true if it is considered close enough to be the same.
	 *
	 * @param expectedStartPosition
	 * @return if it is matching
	 */
	public boolean matchesStartPosition(StartPosition expectedStartPosition) {
		return (MathUtil.isNear(expectedStartPosition.startPose.getX(), startPose2d.getX(), .05)
				&& MathUtil.isNear(expectedStartPosition.startPose.getY(), startPose2d.getY(), .05)
				&& MathUtil.isNear(
						expectedStartPosition.startPose.getRotation().getDegrees(),
						startPose2d.getRotation().getDegrees(),
						5));
	}
}
