package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.IntakeSubsystem;

public class AllInNoSensorCommand extends Command {
	private final IntakeSubsystem intakeSubsystem;

	public AllInNoSensorCommand(IntakeSubsystem intakeSubsystem) {
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
	public boolean isFinished() {
		return true;
	}
}
