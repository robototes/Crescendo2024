package frc.team2412.robot.commands.diagnostic;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.commands.launcher.SetAngleLaunchCommand;
import frc.team2412.robot.commands.launcher.StopLauncherCommand;
import frc.team2412.robot.subsystems.LauncherSubsystem;

public class LauncherDiagnosticCommand extends SequentialCommandGroup {
	public LauncherDiagnosticCommand(LauncherSubsystem launcherSubsystem) {
		double angle = launcherSubsystem.getAngle();
		addCommands(
				new SetAngleLaunchCommand(launcherSubsystem, 500, angle + 10),
				new WaitCommand(2),
				new SetAngleLaunchCommand(launcherSubsystem, 500, angle + 20),
				new WaitCommand(1),
				new SetAngleLaunchCommand(launcherSubsystem, 500, angle),
				new WaitCommand(5),
				new StopLauncherCommand(launcherSubsystem));
	}
}
