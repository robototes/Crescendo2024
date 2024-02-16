package frc.team2412.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.IntakeSubsystem;
import frc.team2412.robot.subsystems.LauncherSubsystem;

public class LaunchCommand extends Command {
	private final LauncherSubsystem launcherSubsystem;
	private final IntakeSubsystem intakeSubsystem;

	public LaunchCommand(LauncherSubsystem launcherSubsystem, IntakeSubsystem intakeSubsystem) {
		this.launcherSubsystem = launcherSubsystem;
		this.intakeSubsystem = intakeSubsystem;
		addRequirements(launcherSubsystem, intakeSubsystem);
	}

	@Override
	public void initialize() {
		intakeSubsystem.feederIn();
		launcherSubsystem.launch();
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void end(boolean interrupted) {
		intakeSubsystem.feederStop();
		launcherSubsystem.stopLauncher();
	}
}
