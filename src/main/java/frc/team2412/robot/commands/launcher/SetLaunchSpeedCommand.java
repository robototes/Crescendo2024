package frc.team2412.robot.commands.launcher;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.LauncherSubsystem;
// this command activates the launcher

public class SetLaunchSpeedCommand extends Command {
	private final LauncherSubsystem launcherSubsystem;
	private final DoubleSupplier launcherSpeed;

	public SetLaunchSpeedCommand(LauncherSubsystem launcherSubsystem, DoubleSupplier speed) {
		launcherSpeed = speed;
		this.launcherSubsystem = launcherSubsystem;
		addRequirements(launcherSubsystem);
	}

	@Override
	public void execute() {
		launcherSubsystem.launch(launcherSpeed.getAsDouble());
	}
	
	@Override
	public void end(boolean Interrupted){
		launcherSubsystem.stopLauncher();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
