package frc.team2412.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.LauncherSubsystem;

public class StopLauncherCommand extends Command {
	private final LauncherSubsystem launcherSubsystem;

	public StopLauncherCommand(LauncherSubsystem launcherSubsystem) {
		this.launcherSubsystem = launcherSubsystem;
		addRequirements(launcherSubsystem);
	}

	@Override
	public void initialize() {
		launcherSubsystem.stopLauncher();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
