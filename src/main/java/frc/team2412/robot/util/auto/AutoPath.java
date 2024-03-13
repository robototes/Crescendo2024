package frc.team2412.robot.util.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.util.auto.AutoLogic.StartPosition;

public class AutoPath {

	private Pose2d startPose2d;
	private StartPosition startPosition;
	private String displayName;
	private Command autoCommand;
	private boolean vision;

	public AutoPath(String displayName, String pathPlannerAutoName, boolean vision) {
		this.displayName = displayName;
		startPose2d = PathPlannerAuto.getStaringPoseFromAutoFile(pathPlannerAutoName);
		autoCommand = AutoBuilder.buildAuto(pathPlannerAutoName);
		this.vision = vision;

		for (StartPosition pos : StartPosition.values()) {
			if (startPose2d.equals(pos.startPose)) {
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
}
