package frc.team2412.robot.commands.diagnostic;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.commands.launcher.SetAngleCommand;
import frc.team2412.robot.commands.launcher.SetLaunchSpeedCommand;
import frc.team2412.robot.commands.launcher.StopLauncherCommand;
import frc.team2412.robot.subsystems.LauncherSubsystem;

public class LauncherDiagnosticCommand extends SequentialCommandGroup {
	private final LauncherSubsystem launcherSubsystem;
	private final double Angle;

	public LauncherDiagnosticCommand(LauncherSubsystem launcherSubsystem) {
		this.launcherSubsystem = launcherSubsystem;
		this.Angle = launcherSubsystem.getAngle();
		addCommands(
				new SetAngleCommand(launcherSubsystem, 45),
				new WaitCommand(2),
				new SetAngleCommand(launcherSubsystem, 90),
				new WaitCommand(1),
				new SetAngleCommand(launcherSubsystem, Angle),
				new SetLaunchSpeedCommand(launcherSubsystem, 100).withTimeout(3),
				new StopLauncherCommand(launcherSubsystem));
	}
}
