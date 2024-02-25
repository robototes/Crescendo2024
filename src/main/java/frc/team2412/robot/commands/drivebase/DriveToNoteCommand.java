package frc.team2412.robot.commands.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.DrivebaseSubsystem;
import frc.team2412.robot.subsystems.LimelightSubsystem;

public class DriveToNoteCommand extends Command {

	private final DrivebaseSubsystem drivebaseSubsystem;
	private final LimelightSubsystem limelightSubsystem;

	private final double INCHES_TO_METERS = 39.3700787;

	public DriveToNoteCommand(
		DrivebaseSubsystem drivebaseSubsystem, LimelightSubsystem limelightSubsystem) {
		this.drivebaseSubsystem = drivebaseSubsystem;
		this.limelightSubsystem = limelightSubsystem;
		addRequirements(drivebaseSubsystem);
	}

	@Override
	public void execute() {
		Translation2d move =
				new Translation2d(-limelightSubsystem.getDistanceFromTarget() / INCHES_TO_METERS, 0.0);
		Rotation2d turn = new Rotation2d().fromDegrees(-2 * limelightSubsystem.getHorizontalOffset());
		drivebaseSubsystem.drive(move, turn, false);
	}

	@Override
	public boolean isFinished() {
		return (limelightSubsystem.getDistanceFromTarget() <= 20);
	}
}
