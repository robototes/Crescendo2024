package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.IntakeSubsystem;

public class FeederInCommand extends Command {
	private final IntakeSubsystem intakeSubsystem;

	public FeederInCommand(IntakeSubsystem intakeSubsystem) {
		this.intakeSubsystem = intakeSubsystem;
		addRequirements(intakeSubsystem);
	}

	@Override
	public void initialize() {
		intakeSubsystem.feederIn();
	}

	@Override
	public void end(boolean interrupted) {
		intakeSubsystem.feederStop();
	}

	@Override
	public boolean isFinished() {
		return intakeSubsystem.feederSensor();
	}
}
