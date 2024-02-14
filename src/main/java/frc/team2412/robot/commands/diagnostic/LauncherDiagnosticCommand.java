package frc.team2412.robot.commands.diagnostic;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.commands.launcher.SetAngleCommand;
import frc.team2412.robot.commands.launcher.SetLaunchSpeedCommand;
import frc.team2412.robot.subsystems.LauncherSubsystem;

public class LauncherDiagnosticCommand extends SequentialCommandGroup {
	private final double angle;

	public LauncherDiagnosticCommand(LauncherSubsystem launcherSubsystem) {
		this.angle = launcherSubsystem.getAngle();
		addCommands(
				new SetAngleCommand(launcherSubsystem, 45).withTimeout(2),
				new WaitCommand(5),
				new SetAngleCommand(launcherSubsystem, 70).withTimeout(2),
				new WaitCommand(5),
				new SetAngleCommand(launcherSubsystem, angle).withTimeout(2),
				new WaitCommand(5),
				new SetLaunchSpeedCommand(launcherSubsystem, 100).withTimeout(2));
	}
}
