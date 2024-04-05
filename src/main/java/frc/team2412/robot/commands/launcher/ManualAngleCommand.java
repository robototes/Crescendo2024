package frc.team2412.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.LauncherSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
// this command adjusts the angle using a speed that is input by the user

public class ManualAngleCommand extends Command {
	private final LauncherSubsystem launcherSubsystem;
	private final DoubleSupplier launcherAngle;
	private final BooleanSupplier powerControl;
	private final BooleanSupplier ignoreLimits;

	public ManualAngleCommand(
			LauncherSubsystem launcherSubsystem,
			DoubleSupplier angleSpeed,
			BooleanSupplier powerControl,
			BooleanSupplier ignoreLimits) {
		this.launcherSubsystem = launcherSubsystem;
		this.launcherAngle = angleSpeed;
		this.powerControl = powerControl;
		this.ignoreLimits = ignoreLimits;
		addRequirements(launcherSubsystem);
	}

	@Override
	public void execute() {
		launcherSubsystem.setAngleManual(launcherAngle.getAsDouble(), powerControl.getAsBoolean(), ignoreLimits.getAsBoolean());
	}

	@Override
	public void end(boolean interrupted) {
		launcherSubsystem.restoreLimits();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
