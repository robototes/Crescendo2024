package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.IntakeSubsystem;

public class NoteStealCommand extends Command {

	private final IntakeSubsystem intakeSubsystem;

	public NoteStealCommand(IntakeSubsystem intakeSubsystem) {
		this.intakeSubsystem = intakeSubsystem;
	}

	@Override
	public void initialize() {
		intakeSubsystem.intakeSteal();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
