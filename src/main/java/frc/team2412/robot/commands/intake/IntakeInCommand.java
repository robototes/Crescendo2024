package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.IntakeSubsystem;

public class IntakeInCommand extends Command {
	private final IntakeSubsystem intakeSubsystem;

	public IntakeInCommand(IntakeSubsystem intakeSubsystem) {
		this.intakeSubsystem = intakeSubsystem;
		addRequirements(intakeSubsystem);
	}

	@Override
	public void initialize() {
		intakeSubsystem.intakeIn();
	}

	@Override
	public void end(boolean interrupted) {
		intakeSubsystem.intakeStop();
	}

	@Override
	public boolean isFinished() {
		return intakeSubsystem.getFeederSensor();
	}
}
