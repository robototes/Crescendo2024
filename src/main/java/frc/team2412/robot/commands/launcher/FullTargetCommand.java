package frc.team2412.robot.commands.launcher;

import java.nio.file.FileSystems;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team2412.robot.Controls;
import frc.team2412.robot.Robot;
import frc.team2412.robot.subsystems.DrivebaseSubsystem;
import frc.team2412.robot.subsystems.LauncherSubsystem;
import frc.team2412.robot.util.LauncherDataLoader;
import frc.team2412.robot.util.LauncherDataPoint;

public class FullTargetCommand extends Command {

	private static final InterpolatingTreeMap<Double, LauncherDataPoint> LAUNCHER_DATA =
			LauncherDataLoader.fromCSV(
					FileSystems.getDefault()
							.getPath(Filesystem.getDeployDirectory().getPath(), "launcher_data.csv"));
	;
	private static final float YAW_TARGET_VIBRATION_TOLERANCE = 3; // degrees
	private final Pose2d SPEAKER_POSE = new Pose2d(0.0, 5.55, Rotation2d.fromRotations(0));
	private final Controls controls = Robot.getInstance().controls;

	private DrivebaseSubsystem drivebaseSubsystem;
	private LauncherSubsystem launcherSubsystem;
	private Command yawAlignmentCommand;
	private Rotation2d yawTarget;
	private BooleanSupplier launch;

	public FullTargetCommand(
			LauncherSubsystem launcherSubsystem, DrivebaseSubsystem drivebaseSubsystem, BooleanSupplier launch) {
		this.launcherSubsystem = launcherSubsystem;
		this.drivebaseSubsystem = drivebaseSubsystem;
		this.launch = launch;
		yawAlignmentCommand = drivebaseSubsystem.rotateToAngle(() -> yawTarget, false);
	}

	@Override
	public void initialize() {
		CommandScheduler.getInstance().schedule(yawAlignmentCommand);
		launcherSubsystem.stopLauncher();
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
		if (launch.getAsBoolean()) {
			launcherSubsystem.setAngle(dataPoint.angle);
			launcherSubsystem.launch(dataPoint.rpm);
			controls.vibrateDriveController(0.0);
		} else if (MathUtil.isNear(yawTarget.getRadians(), drivebaseSubsystem.getPose().getRotation().getRadians(), YAW_TARGET_VIBRATION_TOLERANCE)) {
			controls.vibrateDriveController(1.0);
		} else {
			controls.vibrateDriveController(0.0);
		}
	}

	@Override
	public void end(boolean interrupted) {
		yawAlignmentCommand.cancel();
		launcherSubsystem.stopLauncher();
		controls.vibrateDriveController(0.0);
	}
}
