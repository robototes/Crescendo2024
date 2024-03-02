package frc.team2412.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.LauncherSubsystem;
import java.util.function.DoubleSupplier;
// this command adjusts the angle using a speed that is input by the user

public class SetAngleCommand extends Command {
	private final LauncherSubsystem launcherSubsystem;
	private final DoubleSupplier launcherAngleSpeed;

	public SetAngleCommand(LauncherSubsystem launcherSubsystem, DoubleSupplier angleSpeed) {
		this.launcherSubsystem = launcherSubsystem;
		this.launcherAngleSpeed = angleSpeed;
		addRequirements(launcherSubsystem);
	}

	@Override
	public void execute() {
		if (launcherAngleSpeed.getAsDouble() != 0.0) {
			launcherSubsystem.setAngleSpeed(launcherAngleSpeed.getAsDouble());
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
