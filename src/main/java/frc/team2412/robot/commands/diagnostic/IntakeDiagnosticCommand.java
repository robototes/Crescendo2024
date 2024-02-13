package frc.team2412.robot.commands.diagnostic;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.commands.intake.FeederInCommand;
import frc.team2412.robot.commands.intake.FeederOutCommand;
import frc.team2412.robot.commands.intake.FeederStopCommand;
import frc.team2412.robot.commands.intake.IndexInCommand;
import frc.team2412.robot.commands.intake.IndexOutCommand;
import frc.team2412.robot.commands.intake.IndexStopCommand;
import frc.team2412.robot.commands.intake.IntakeInCommand;
import frc.team2412.robot.commands.intake.IntakeOutCommand;
import frc.team2412.robot.commands.intake.IntakeStopCommand;
import frc.team2412.robot.subsystems.IntakeSubsystem;

public class IntakeDiagnosticCommand extends SequentialCommandGroup {
	private final IntakeSubsystem intakeSubsystem;

	public IntakeDiagnosticCommand(IntakeSubsystem intakeSubsystem) {
		this.intakeSubsystem = intakeSubsystem;
		addCommands(
				new FeederInCommand(intakeSubsystem).withTimeout(0.5),
				new WaitCommand(0.5),
				new FeederOutCommand(intakeSubsystem),
				new WaitCommand(0.5),
				new FeederStopCommand(intakeSubsystem),
				new WaitCommand(0.5),
				new IndexInCommand(intakeSubsystem).withTimeout(0.5),
				new WaitCommand(0.5),
				new IndexOutCommand(intakeSubsystem),
				new WaitCommand(0.5),
				new IndexStopCommand(intakeSubsystem),
				new WaitCommand(0.5),
				new IntakeInCommand(intakeSubsystem).withTimeout(0.5),
				new WaitCommand(0.5),
				new IntakeOutCommand(intakeSubsystem),
				new WaitCommand(0.5),
				new IntakeStopCommand(intakeSubsystem));
	}
}
