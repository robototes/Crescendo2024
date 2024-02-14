package frc.team2412.robot.commands.launcher;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team2412.robot.subsystems.DrivebaseSubsystem;
import frc.team2412.robot.subsystems.LauncherSubsystem;
import frc.team2412.robot.util.LauncherDataLoader;
import frc.team2412.robot.util.LauncherDataPoint;
import java.nio.file.FileSystems;

public class FullTargetCommand extends Command {

	private final static InterpolatingTreeMap<Double, LauncherDataPoint> LAUNCHER_DATA = LauncherDataLoader.fromCSV(
		FileSystems.getDefault()
				.getPath(Filesystem.getDeployDirectory().getPath(), "launcher_data.csv"));;
	private final Pose2d SPEAKER_POSE = new Pose2d(0.0, 5.55, Rotation2d.fromRotations(0));
	
	private DrivebaseSubsystem drivebaseSubsystem;
	private LauncherSubsystem launcherSubsystem;
	private Command yawAlignmentCommand;
	private Rotation2d yawTarget;

	public FullTargetCommand(
			LauncherSubsystem launcherSubsystem, DrivebaseSubsystem drivebaseSubsystem) {
		this.launcherSubsystem = launcherSubsystem;
		this.drivebaseSubsystem = drivebaseSubsystem;
		yawAlignmentCommand = drivebaseSubsystem.rotateToAngle(() -> yawTarget, false);				
	}

	@Override
	public void initialize() {
		CommandScheduler.getInstance().schedule(yawAlignmentCommand);
	}

	@Override
	public void execute() {
		Pose2d robotPose = drivebaseSubsystem.getPose();
		Pose2d relativeSpeaker = robotPose.relativeTo(SPEAKER_POSE);
		yawTarget =
				Rotation2d.fromRadians(
						Math.atan2(relativeSpeaker.getY(), relativeSpeaker.getX()) + Math.PI);
		double distance = relativeSpeaker.getTranslation().getNorm();
		LauncherDataPoint dataPoint = LAUNCHER_DATA.get(distance);
		launcherSubsystem.setAngle(dataPoint.angle);
		launcherSubsystem.launch(dataPoint.rpm);
	}

	@Override
	public void end(boolean interrupted) {
		yawAlignmentCommand.cancel();
	}
}
