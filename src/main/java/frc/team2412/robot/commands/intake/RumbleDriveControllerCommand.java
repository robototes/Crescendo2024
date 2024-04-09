package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.Controls;
import frc.team2412.robot.Controls.ControlConstants;

public class RumbleDriveControllerCommand extends Command {
	private final Controls controls;

	public RumbleDriveControllerCommand(Controls controls) {
		this.controls = controls;
	}

	@Override
	public void initialize() {
		controls.vibrateDriveController(ControlConstants.RUMBLE_VIBRATION);
	}
	// exist
	@Override
	public void end(boolean interrupted) {
		controls.vibrateDriveController(0.0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
