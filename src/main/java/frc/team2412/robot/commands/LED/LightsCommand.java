package frc.team2412.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.IntakeSubsystem;
import frc.team2412.robot.subsystems.LEDSubsystem;
import frc.team2412.robot.subsystems.LauncherSubsystem;

public class LightsCommand extends Command {
	private final LEDSubsystem ledSubsystem;
	private final IntakeSubsystem intakeSubsystem;
	private final LauncherSubsystem launcherSubsystem;

	public LightsCommand(
			LEDSubsystem ledSubsystem,
			IntakeSubsystem intakeSubsystem,
			LauncherSubsystem launcherSubsystem) {
		this.ledSubsystem = ledSubsystem;
		this.intakeSubsystem = intakeSubsystem;
		this.launcherSubsystem = launcherSubsystem;
		addRequirements(ledSubsystem);
	}

	@Override
	public void initialize() {
		ledSubsystem.setRED_LED();
	}

	@Override
	public void execute() {
		if (launcherSubsystem.isAtSpeed()) { // Checks if launcher is ready
			ledSubsystem.setGREEN_LED();
		} else if (intakeSubsystem.indexSensorHasNote()) { // Checks if note is in feeder
			ledSubsystem.setBLUE_LED();
		} else if (intakeSubsystem.isIntakeOn()) { // Checks if intake is on
			ledSubsystem.setYELLOW_LED();
		} else { // Everything else including intake off
			ledSubsystem.setRED_LED();
		}
	}

	@Override
	public void end(boolean interrupted) {
		ledSubsystem.disableLED();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
