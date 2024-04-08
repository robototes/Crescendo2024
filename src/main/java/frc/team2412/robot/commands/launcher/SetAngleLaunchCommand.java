package frc.team2412.robot.commands.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.Controls;
import frc.team2412.robot.Robot;
import frc.team2412.robot.commands.intake.RumbleCommand;
import frc.team2412.robot.subsystems.LauncherSubsystem;
// this command can be used as a preset in controls, allowing the user to input a speed and angle
// value when they keybind it multiple times.

public class SetAngleLaunchCommand extends Command {
	private double launcherSpeed;
	private double launcherAngle;
	private LauncherSubsystem launcherSubsystem;

	public SetAngleLaunchCommand(LauncherSubsystem launcherSubsystem, double speed, double angle) {
		launcherSpeed = speed;
		launcherAngle = angle;
		this.launcherSubsystem = launcherSubsystem;
		addRequirements(launcherSubsystem);
	}

	@Override
	public void initialize() {
		launcherSubsystem.setAngle(launcherAngle);
		launcherSubsystem.launch(launcherSpeed);
	}

	@Override
	public boolean isFinished() {
		return MathUtil.isNear(
				launcherAngle, launcherSubsystem.getAngle(), LauncherSubsystem.ANGLE_TOLERANCE);
	}

	@Override
	public void end(boolean interrupted) {
		Robot robot = Robot.getInstance();
		Controls controls = robot.controls;
		Commands.race(new RumbleCommand(controls), new WaitCommand(1)).schedule();
	}
}
