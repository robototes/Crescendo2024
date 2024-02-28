package frc.team2412.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2412.robot.Controls;
import frc.team2412.robot.subsystems.IntakeSubsystem;

public class AllInCommand extends Command {
	private final IntakeSubsystem intakeSubsystem;
	private final Controls controls;

	public AllInCommand(IntakeSubsystem intakeSubsystem, Controls controls) {
		this.intakeSubsystem = intakeSubsystem;
		this.controls = controls;
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
		intakeSubsystem.intakeReject();
		intakeSubsystem.indexStop();
		intakeSubsystem.feederStop();

		if (controls != null && interrupted == false) {
			controls.vibrateDriveController(1.0);
		}
	}

	@Override
	public boolean isFinished() {
		return intakeSubsystem.feederSensorHasNote();
	}
}
