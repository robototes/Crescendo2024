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
		if (intakeSubsystem.getIndexSensor()) {
			ledSubsystem.setBLUE_LED();
		} else if (launcherSubsystem.atTargetSpeed()) {
			ledSubsystem.setGREEN_LED();
		}
	}

	@Override
	public boolean isFinished() {
		ledSubsystem.setRED_LED();
		return true;
	}
}
