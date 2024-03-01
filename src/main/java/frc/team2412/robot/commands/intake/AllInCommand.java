package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.IntakeSubsystem;

public class AllInCommand extends Command {
	private final IntakeSubsystem intakeSubsystem;

	public AllInCommand(IntakeSubsystem intakeSubsystem) {
		this.intakeSubsystem = intakeSubsystem;
		addRequirements(intakeSubsystem);
	}

	@Override
	public void initialize() {
		intakeSubsystem.intakeIn();
		intakeSubsystem.indexIn();
		intakeSubsystem.feederIn();
	}

	@Override
	public void execute() {
		if (intakeSubsystem.indexSensorHasNote()) {
			intakeSubsystem.intakeReject();
		}
	}

	@Override
	public void end(boolean interrupted) {
		// intakeSubsystem.intakeReject();
		intakeSubsystem.indexStop();
		intakeSubsystem.feederStop();
	}

	@Override
	public boolean isFinished() {
		return intakeSubsystem.feederSensorHasNote();
	}
}
