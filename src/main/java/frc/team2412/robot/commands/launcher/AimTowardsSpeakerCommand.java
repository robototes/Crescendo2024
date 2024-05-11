package frc.team2412.robot.commands.launcher;

import static frc.team2412.robot.Subsystems.SubsystemConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team2412.robot.subsystems.DrivebaseSubsystem;
import frc.team2412.robot.subsystems.LauncherSubsystem;
import frc.team2412.robot.util.LauncherDataPoint;

public class AimTowardsSpeakerCommand extends Command {

	private final double HEADING_TOLERANCE = 2.0;

	private Translation2d SPEAKER_POSITION;

	private DrivebaseSubsystem drivebaseSubsystem;
	private LauncherSubsystem launcherSubsystem;
	private Command yawAlignmentCommand;
	private Rotation2d yawTarget = new Rotation2d();

	public AimTowardsSpeakerCommand(
			LauncherSubsystem launcherSubsystem, DrivebaseSubsystem drivebaseSubsystem) {
		this.launcherSubsystem = launcherSubsystem;
		this.drivebaseSubsystem = drivebaseSubsystem;
		if (DRIVEBASE_ENABLED) {
			yawAlignmentCommand = drivebaseSubsystem.forceRotateToAngle(() -> yawTarget);
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
		double distance = robotToSpeaker.getNorm();
		LauncherDataPoint dataPoint = FullTargetCommand.LAUNCHER_DATA.get(distance);
		launcherSubsystem.updateDistanceEntry(distance);

		yawTarget = robotToSpeaker.getAngle();
		launcherSubsystem.setAngleWithOffset(dataPoint.angle);
	}

	@Override
	public void end(boolean interrupted) {
		yawAlignmentCommand.cancel();
	}

	@Override
	public boolean isFinished() {
		return (MathUtil.isNear(
						drivebaseSubsystem.getPose().getRotation().getDegrees(),
						yawTarget.getDegrees(),
						HEADING_TOLERANCE)
				&& launcherSubsystem.isAtAngle());
	}
}
