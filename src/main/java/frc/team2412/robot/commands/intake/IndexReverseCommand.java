package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.IntakeSubsystem;

public class IndexReverseCommand extends Command {
	private final IntakeSubsystem intakeSubsystem;

	public IndexReverseCommand(IntakeSubsystem intakeSubsystem) {
		this.intakeSubsystem = intakeSubsystem;
		addRequirements(intakeSubsystem);
	}

	@Override
	public void initialize() {
		intakeSubsystem.indexReverse();
	}

	public boolean isFinished() {
		return true;
	}
}
