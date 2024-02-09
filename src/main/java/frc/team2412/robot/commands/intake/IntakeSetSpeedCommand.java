package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.IntakeSubsystem;

public class IntakeSetSpeedCommand extends Command {
	private final IntakeSubsystem intakeSubsystem;
	private double intakeSpeed;

	public IntakeSetSpeedCommand(IntakeSubsystem intakeSubsystem, double intakeSpeed) {
		this.intakeSubsystem = intakeSubsystem;
		this.intakeSpeed = intakeSpeed;
		addRequirements(intakeSubsystem);
	}

	@Override
	public void initialize() {
		intakeSubsystem.intakeIn();
	}
}
