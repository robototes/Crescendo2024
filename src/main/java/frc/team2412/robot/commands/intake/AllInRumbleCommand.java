package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.Controls;
import frc.team2412.robot.subsystems.IntakeSubsystem;

public class AllInRumbleCommand extends SequentialCommandGroup {
	private final IntakeSubsystem intakeSubsystem;
	private final Controls controls;

	public AllInRumbleCommand(IntakeSubsystem intakeSubsystem, Controls controls) {
		this.intakeSubsystem = intakeSubsystem;
		this.controls = controls;
		addRequirements(intakeSubsystem);

		addCommands(new AllInCommand(intakeSubsystem, controls), new WaitCommand(0.5));
	}

	public void end() {
		controls.vibrateDriveController(0.0);
	}
}
