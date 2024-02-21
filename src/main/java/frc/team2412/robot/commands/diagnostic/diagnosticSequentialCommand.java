package frc.team2412.robot.commands.diagnostic;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.subsystems.IntakeSubsystem;
import frc.team2412.robot.subsystems.LauncherSubsystem;

public class diagnosticSequentialCommand extends SequentialCommandGroup {
	private final LauncherSubsystem launcherSubsystem;
	private final IntakeSubsystem intakeSubsystem;

	public diagnosticSequentialCommand(
			IntakeSubsystem intakeSubsystem, LauncherSubsystem launcherSubsystem) {
		this.launcherSubsystem = launcherSubsystem;
		this.intakeSubsystem = intakeSubsystem;

		addCommands(
				new IntakeDiagnosticCommand(intakeSubsystem),
				new WaitCommand(1),
				new LauncherDiagnosticCommand(launcherSubsystem));
	}
}
