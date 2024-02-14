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

	private final Pose2d SPEAKER_POSE = new Pose2d(0.0, 5.55, Rotation2d.fromRotations(0));

	private Rotation2d yawAngle;
	private double angle;
	private double rpm;
	private Pose2d robotPose;
	private Pose2d relativeSpeaker;
	private double distance;
	private LauncherDataPoint dataPoint;
	private Command setAngleCommand;
	private Command setLaunchSpeedCommand;
	private Command rotateToAngle;
	private InterpolatingTreeMap<Double, LauncherDataPoint> launcherData;

	DrivebaseSubsystem drivebaseSubsystem;
	LauncherSubsystem launcherSubsystem;

	public FullTargetCommand(
			LauncherSubsystem launcherSubsystem, DrivebaseSubsystem drivebaseSubsystem) {
		this.launcherSubsystem = launcherSubsystem;
		this.drivebaseSubsystem = drivebaseSubsystem;
		setLaunchSpeedCommand = new SetLaunchSpeedCommand(launcherSubsystem, () -> rpm);
		setAngleCommand = new SetAngleCommand(launcherSubsystem, () -> angle);
		rotateToAngle = drivebaseSubsystem.rotateToAngle(() -> yawAngle, false);
		launcherData =
				LauncherDataLoader.fromCSV(
						FileSystems.getDefault()
								.getPath(Filesystem.getDeployDirectory().getPath(), "launcher_data.csv"));
	}

	@Override
	public void initialize() {
		CommandScheduler.getInstance().schedule(setAngleCommand);
		CommandScheduler.getInstance().schedule(setLaunchSpeedCommand);
		CommandScheduler.getInstance().schedule(rotateToAngle);
	}

	@Override
	public void execute() {
		robotPose = drivebaseSubsystem.getPose();
		relativeSpeaker = robotPose.relativeTo(SPEAKER_POSE);
		yawAngle =
				Rotation2d.fromRadians(
						Math.atan2(relativeSpeaker.getY(), relativeSpeaker.getX()) + Math.PI);
		distance = relativeSpeaker.getTranslation().getNorm();
		dataPoint = launcherData.get(distance);
		rpm = dataPoint.rpm;
		angle = dataPoint.angle;
	}

	@Override
	public void end(boolean interrupted) {
		rotateToAngle.cancel();
		setLaunchSpeedCommand.cancel();
		setAngleCommand.cancel();
	}
}
