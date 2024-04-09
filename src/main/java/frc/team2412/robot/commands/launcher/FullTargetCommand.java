package frc.team2412.robot.commands.launcher;

import static frc.team2412.robot.Subsystems.SubsystemConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team2412.robot.Controls;
import frc.team2412.robot.Controls.ControlConstants;
import frc.team2412.robot.subsystems.DrivebaseSubsystem;
import frc.team2412.robot.subsystems.LauncherSubsystem;
import frc.team2412.robot.util.LauncherDataLoader;
import frc.team2412.robot.util.LauncherDataPoint;
import java.nio.file.FileSystems;

public class FullTargetCommand extends Command {

	private static final InterpolatingTreeMap<Double, LauncherDataPoint> LAUNCHER_DATA =
			LauncherDataLoader.fromCSV(
					FileSystems.getDefault()
							.getPath(
									Filesystem.getDeployDirectory().getPath(),
									LauncherSubsystem.USE_THROUGHBORE
											? "launcher_data_throughbore.csv"
											: "launcher_data_lamprey.csv"));
	private static final double YAW_TARGET_VIBRATION_TOLERANCE = 10; // degrees
	private Translation2d SPEAKER_POSITION;

	private DrivebaseSubsystem drivebaseSubsystem;
	private LauncherSubsystem launcherSubsystem;
	private Command yawAlignmentCommand;
	private Rotation2d yawTarget;
	private Controls controls;

	public FullTargetCommand(
			LauncherSubsystem launcherSubsystem,
			DrivebaseSubsystem drivebaseSubsystem,
			Controls controls) {
		this.launcherSubsystem = launcherSubsystem;
		this.drivebaseSubsystem = drivebaseSubsystem;
		this.controls = controls;
		if (DRIVEBASE_ENABLED) {
			yawAlignmentCommand = drivebaseSubsystem.rotateToAngle(() -> yawTarget, false);
		}

		addRequirements(launcherSubsystem);
	}

	@Override
	public void initialize() {
		CommandScheduler.getInstance().schedule(yawAlignmentCommand);

		SPEAKER_POSITION =
				DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue
						? new Translation2d(0.0, 5.55)
						: new Translation2d(16.5, 5.55);
	}

	@Override
	public void execute() {
		// look ahead half a second into the future
		var fieldSpeed = drivebaseSubsystem.getFieldSpeeds().times(0.5);
		Translation2d robotPosition =
				drivebaseSubsystem
						.getPose()
						.getTranslation()
						.plus(new Translation2d(fieldSpeed.vxMetersPerSecond, fieldSpeed.vyMetersPerSecond));
		Translation2d robotToSpeaker = SPEAKER_POSITION.minus(robotPosition);
		yawTarget = robotToSpeaker.getAngle();
		double distance = robotToSpeaker.getNorm();
		LauncherDataPoint dataPoint = LAUNCHER_DATA.get(distance);
		launcherSubsystem.setAngleWithOffset(dataPoint.angle);
		launcherSubsystem.launch(dataPoint.rpm);
		launcherSubsystem.updateDistanceEntry(distance);

		// launcher angle checker
		if (launcherSubsystem.isAtAngle() && launcherSubsystem.isAtSpeed()) {
			controls.vibrateDriveController(ControlConstants.RUMBLE_VIBRATION);
		} else {
			controls.vibrateDriveController(0.0);
		}
	}

	@Override
	public void end(boolean interrupted) {
		yawAlignmentCommand.cancel();
		// launcherSubsystem.launch(2000);
		launcherSubsystem.stopLauncher();
		controls.vibrateDriveController(0.0);
	}
}
