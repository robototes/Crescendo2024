package frc.team2412.robot.commands.launcher;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.WaitCommand;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.subsystems.LauncherSubsystem;
// this command can be used as a preset in controls, allowing the user to input a speed and angle
// value when they keybind it multiple times.

public class SetAngleLaunchCommand extends SequentialCommandGroup {
	private double launcherSpeed;
	private double launcherAngle;
	private Command waitCommand = new waitCommand(2);

	public SetAngleLaunchCommand(LauncherSubsystem launcherSubsystem, double speed, double angle) {
		launcherSpeed = speed;
		launcherAngle = angle;
		addCommands(
				new SetAngleCommand(launcherSubsystem, launcherAngle),
				waitCommand,
				new SetLaunchSpeedCommand(launcherSubsystem, launcherSpeed));
	}
}
