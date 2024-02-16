package frc.team2412.robot.commands.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team2412.robot.Controls;
import frc.team2412.robot.subsystems.DrivebaseSubsystem;
import frc.team2412.robot.subsystems.IntakeSubsystem;
import frc.team2412.robot.subsystems.LauncherSubsystem;
import frc.team2412.robot.util.LauncherDataLoader;
import frc.team2412.robot.util.LauncherDataPoint;
import java.nio.file.FileSystems;
import java.util.function.BooleanSupplier;

public class FullTargetCommand extends Command {

	private static final InterpolatingTreeMap<Double, LauncherDataPoint> LAUNCHER_DATA =
			LauncherDataLoader.fromCSV(
					FileSystems.getDefault()
							.getPath(Filesystem.getDeployDirectory().getPath(), "launcher_data.csv"));
	private static final double YAW_TARGET_VIBRATION_TOLERANCE = 3; // degrees
	private final Pose2d SPEAKER_POSE = new Pose2d(0.0, 5.55, Rotation2d.fromRotations(0));

	private DrivebaseSubsystem drivebaseSubsystem;
	private LauncherSubsystem launcherSubsystem;
	private IntakeSubsystem intakeSubsystem;
	private Command yawAlignmentCommand;
	private Rotation2d yawTarget;
	private BooleanSupplier launch;
	private Controls controls;

	public FullTargetCommand(
			LauncherSubsystem launcherSubsystem,
			IntakeSubsystem intakeSubsystem,
			DrivebaseSubsystem drivebaseSubsystem,
			Controls controls,
			BooleanSupplier launch) {
		this.launcherSubsystem = launcherSubsystem;
		this.intakeSubsystem = intakeSubsystem;
		this.drivebaseSubsystem = drivebaseSubsystem;
		this.controls = controls;
		this.launch = launch;
		yawAlignmentCommand = drivebaseSubsystem.rotateToAngle(() -> yawTarget, false);

		addRequirements(launcherSubsystem, intakeSubsystem);
	}

	@Override
	public void initialize() {
		CommandScheduler.getInstance().schedule(yawAlignmentCommand);
		intakeSubsystem.feederStop();
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

		if (launch.getAsBoolean()) {
			intakeSubsystem.feederIn();
		} else {
			intakeSubsystem.feederStop();
		}

		if (MathUtil.isNear(
						yawTarget.getDegrees(),
						drivebaseSubsystem.getPose().getRotation().getDegrees(),
						YAW_TARGET_VIBRATION_TOLERANCE)
				&& launcherSubsystem.isAtAngle()
				&& launcherSubsystem.isAtSpeed()) {
			controls.vibrateDriveController(1.0);
		} else {
			controls.vibrateDriveController(0.0);
		}
	}

	@Override
	public void end(boolean interrupted) {
		yawAlignmentCommand.cancel();
		launcherSubsystem.stopLauncher();
		intakeSubsystem.feederStop();
		controls.vibrateDriveController(0.0);
	}
}
