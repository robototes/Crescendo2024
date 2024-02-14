package frc.team2412.robot.commands.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.LauncherSubsystem;
// this command adjusts the angle to a value that is input by the user.

public class SetAngleCommand extends Command {
	private final LauncherSubsystem launcherSubsystem;
	private final double launcherAngle;

	public SetAngleCommand(LauncherSubsystem launcherSubsystem, double angle) {
		this.launcherSubsystem = launcherSubsystem;
		this.launcherAngle = angle;
		addRequirements(launcherSubsystem);
	}

	@Override
	public void initialize() {
		launcherSubsystem.setAngle(launcherAngle);
	}

	@Override
	public boolean isFinished() {
		return (MathUtil.isNear(
				launcherAngle,
				launcherSubsystem.getAngle(),
				LauncherSubsystem.ANGLE_TOLERANCE));
	}
}
