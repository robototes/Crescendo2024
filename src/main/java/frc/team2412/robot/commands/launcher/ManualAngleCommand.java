package frc.team2412.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.LauncherSubsystem;
import java.util.function.DoubleSupplier;
// this command adjusts the angle using a speed that is input by the user

public class ManualAngleCommand extends Command {
	private final LauncherSubsystem launcherSubsystem;
	private final DoubleSupplier launcherAngle;

	public ManualAngleCommand(LauncherSubsystem launcherSubsystem, DoubleSupplier angleSpeed) {
		this.launcherSubsystem = launcherSubsystem;
		this.launcherAngle = angleSpeed;
		addRequirements(launcherSubsystem);
	}

	@Override
	public void execute() {
		launcherSubsystem.setAngleManual(launcherAngle.getAsDouble());
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
