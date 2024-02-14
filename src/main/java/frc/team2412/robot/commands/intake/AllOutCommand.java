package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.IntakeSubsystem;

public class AllOutCommand extends Command {
	private final IntakeSubsystem intakeSubsystem;

	public AllOutCommand(IntakeSubsystem intakeSubsystem) {
		this.intakeSubsystem = intakeSubsystem;
		addRequirements(intakeSubsystem);
	}

	@Override
	public void initialize() {
		intakeSubsystem.intakeOut();
		intakeSubsystem.indexOut();
		intakeSubsystem.feederOut();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
