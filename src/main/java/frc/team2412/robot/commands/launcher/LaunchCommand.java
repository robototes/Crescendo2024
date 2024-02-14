package frc.team2412.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.IntakeSubsystem;
import frc.team2412.robot.subsystems.LauncherSubsystem;

public class LaunchCommand extends Command {
	private final LauncherSubsystem launcherSubsystem;
	private final IntakeSubsystem IntakeSubsystem;

	public LaunchCommand(LauncherSubsystem launcherSubsystem, IntakeSubsystem IntakeSubsystem) {
		this.launcherSubsystem = launcherSubsystem;
		this.IntakeSubsystem = IntakeSubsystem;
		addRequirements(launcherSubsystem, IntakeSubsystem);
	}

	@Override
	public void initialize() {
		IntakeSubsystem.feederOut();
		launcherSubsystem.launch();
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void end(boolean interrupted) {
		IntakeSubsystem.feederStop();
		launcherSubsystem.stopLauncher();
	}
}
