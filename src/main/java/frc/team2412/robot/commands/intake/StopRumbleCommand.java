package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.Controls;

public class StopRumbleCommand extends Command {
	private final Controls controls;

	public StopRumbleCommand(Controls controls) {
		this.controls = controls;
	}

	@Override
	public void initialize() {
		controls.vibrateDriveController(0.0);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
