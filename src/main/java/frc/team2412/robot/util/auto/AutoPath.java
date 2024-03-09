package frc.team2412.robot.util.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoPath {

	private Pose2d startingPose;
	private String displayName;
	private Command autoCommand;
	private boolean vision;

	public AutoPath(String displayName, String pathPlannerAutoName, boolean vision) {
		this.displayName = displayName;
		startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(pathPlannerAutoName);
		autoCommand = AutoBuilder.buildAuto(pathPlannerAutoName);
		this.vision = vision;
	}
	
	public AutoPath(String displayName, String pathPlannerAutoName) {
		this(displayName, pathPlannerAutoName, false);
	}

	public Pose2d getStartPose() {
		return startingPose;
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
