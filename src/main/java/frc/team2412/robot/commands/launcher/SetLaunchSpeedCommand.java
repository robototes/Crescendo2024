package frc.team2412.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.LauncherSubsystem;
// this command activates the launcher

public class SetLaunchSpeedCommand extends Command {
	private final LauncherSubsystem launcherSubsystem;
	private final double launcherSpeed;

	public SetLaunchSpeedCommand(LauncherSubsystem launcherSubsystem, double speed) {
		launcherSpeed = speed;
		this.launcherSubsystem = launcherSubsystem;
		addRequirements(launcherSubsystem);
	}

	@Override
	public void initialize() {
		launcherSubsystem.launch(launcherSpeed);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted){
		launcherSubsystem.stopLauncher();
	}
}
