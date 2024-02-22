package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.subsystems.IntakeSubsystem;

public class FeederAlignmentCommand extends Command {
	private final IntakeSubsystem intakeSubsystem;
	private boolean goingForward;

	public FeederAlignmentCommand(IntakeSubsystem intakeSubsystem) {
		this.intakeSubsystem = intakeSubsystem;
		addRequirements(intakeSubsystem);

		goingForward = false;
	}

	@Override
	public void initialize() {
		// intakeSubsystem.feederAlignBackward();
	}

	@Override
	public void execute() {
		// if (!intakeSubsystem.getFeederSensor()) {
		// 	intakeSubsystem.feederAlignForward();
		// 	goingForward = true;
		// }
	}

	@Override
	public void end(boolean interrupted) {
		// intakeSubsystem.feederStop();
	}

	@Override
	public boolean isFinished() {
		return intakeSubsystem.getFeederSensor() && goingForward;
	}
}
