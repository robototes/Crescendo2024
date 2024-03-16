package frc.team2412.robot.commands.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.LauncherSubsystem;

public class SetPivotCommand extends Command {
	private final LauncherSubsystem launcherSubsystem;
	private final double angle;

	public SetPivotCommand(LauncherSubsystem launcherSubsystem, double angle) {
		this.launcherSubsystem = launcherSubsystem;
		this.angle = angle;
		addRequirements(launcherSubsystem);
	}

	@Override
	public void initialize() {
		launcherSubsystem.setAngle(angle);
		}

	@Override
	public boolean isFinished() {
		return MathUtil.isNear(angle, launcherSubsystem.getAngle(), LauncherSubsystem.ANGLE_TOLERANCE);
	}
}
