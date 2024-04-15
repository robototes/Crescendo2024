package frc.team2412.robot.commands.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.DrivebaseSubsystem;
import frc.team2412.robot.subsystems.LimelightSubsystem;

public class DriveToNoteCommand extends Command {

	private final DrivebaseSubsystem drivebaseSubsystem;
	private final LimelightSubsystem limelightSubsystem;

	// limelight placement might be different so this multiplier is convenient
	private final double INVERT_DRIVE_DIRECTION = -1.0;

	public DriveToNoteCommand(
			DrivebaseSubsystem drivebaseSubsystem, LimelightSubsystem limelightSubsystem) {
		this.drivebaseSubsystem = drivebaseSubsystem;
		this.limelightSubsystem = limelightSubsystem;
		addRequirements(drivebaseSubsystem);
	}

	@Override
	public void execute() {
		if (limelightSubsystem.hasTargets()) {
			Translation2d move =
					new Translation2d(
							INVERT_DRIVE_DIRECTION
									* Units.inchesToMeters(limelightSubsystem.getDistanceFromTargetInches()),
							0.0);
			Rotation2d turn =
					Rotation2d.fromDegrees(
							2 * INVERT_DRIVE_DIRECTION * limelightSubsystem.getHorizontalOffset());
			drivebaseSubsystem.drive(move, turn, false);
		}
	}

	@Override
	public boolean isFinished() {
		return (limelightSubsystem.isWithinDistance() || !limelightSubsystem.hasTargets());
	}
}
