package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.IntakeSubsystem;

public class IndexInCommand extends Command {
	private final IntakeSubsystem intakeSubsystem;

	public IndexInCommand(IntakeSubsystem intakeSubsystem) {
		this.intakeSubsystem = intakeSubsystem;
		addRequirements(intakeSubsystem);
	}

	@Override
	public void initialize() {
		intakeSubsystem.indexIn();
	}

	@Override
	public void end(boolean interrupted) {
		intakeSubsystem.feederStop();
	}

	@Override
	public boolean isFinished() {
		return intakeSubsystem.getFeederSensor();
	}
}
