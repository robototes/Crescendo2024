package frc.team2412.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team2412.robot.subsystems.LauncherSubsystem;

public class SetAngleLaunchCommand extends ParallelCommandGroup {
	private double launcherSpeed;
	private double launcherAngle;

	public SetAngleLaunchCommand(LauncherSubsystem launcherSubsystem, double speed, double angle) {
		launcherSpeed = speed;
		launcherAngle = angle;
		addCommands(
				new SetAngleCommand(launcherSubsystem, launcherAngle),
				new SetLaunchSpeedCommand(launcherSubsystem, launcherSpeed));
	}
}
