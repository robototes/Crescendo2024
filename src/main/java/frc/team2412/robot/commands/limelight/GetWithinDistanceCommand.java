package frc.team2412.robot.commands.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.DrivebaseSubsystem;
import frc.team2412.robot.subsystems.LimelightSubsystem;

public class GetWithinDistanceCommand extends Command {

	private LimelightSubsystem limelightSubsystem;
	private DrivebaseSubsystem drivebaseSubsystem;
	private Pose2d currentPose;

	private Command getToPositionCommand;

	public GetWithinDistanceCommand(
			LimelightSubsystem limelightSubsystem, DrivebaseSubsystem drivebaseSubsystem) {
		this.limelightSubsystem = limelightSubsystem;
		this.drivebaseSubsystem = drivebaseSubsystem;

		// find target -> get current pose -> use current pose and target to calculate target pose ->
		// move towards target pose

	}

	@Override
	public void initialize() {
		currentPose = drivebaseSubsystem.getPose();
		getToPositionCommand = limelightSubsystem.getWithinDistance(currentPose, drivebaseSubsystem);
		limelightSubsystem.getWithinDistance(currentPose, drivebaseSubsystem);
		getToPositionCommand.initialize();
	}

	@Override
	public void execute() {
		getToPositionCommand.execute();
		limelightSubsystem.getTargetPose(currentPose);
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return limelightSubsystem.isWithinDistance();
	}
}
